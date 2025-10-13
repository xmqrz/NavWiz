from __future__ import absolute_import
from __future__ import unicode_literals

from celery import shared_task
from django.conf import settings as django_settings
import datetime
import glob
import io
import os
import subprocess
import tarfile
import time
import uuid


BUNDLE_DIR = os.path.join(django_settings.MEDIA_ROOT, 'download_log/')
LOG_DIR = (
    '/var/log/kern.log*',
    '/var/log/syslog*',
    '/var/log/navwiz/*',
    '/var/log/navwiz/*/*',
    '/var/log/mysql/*',
    '/var/log/nginx/*',
    '/var/lib/navwiz/.ros/diagnostics/*',
    '/var/lib/navwiz/.ros/log/*/*',
)


@shared_task(ignore_result=True)
def cleanup_log():
    oldest = datetime.datetime(year=1, month=1, day=1).date()
    yesterday = datetime.datetime.now().date() - datetime.timedelta(days=1)
    for file_data in _get_files(oldest, yesterday, BUNDLE_DIR):
        filepath = file_data[1]
        try:
            os.remove(filepath)
        except Exception:
            pass


@shared_task(bind=True)
def bundle_log(self, start_date, end_date):
    filename = 'navwiz_log.tar.gz'
    if start_date == end_date:
        filename = 'navwiz_log_%s.tar.gz' % start_date.strftime("%Y%m%d")
    else:
        filename = 'navwiz_log_%s_%s.tar.gz' % \
            (start_date.strftime("%Y%m%d"), end_date.strftime("%Y%m%d"))

    try:
        os.makedirs(BUNDLE_DIR)
    except Exception:
        pass

    unique_filename = '%s_%s' % (uuid.uuid4(), filename)

    filepath = os.path.join(BUNDLE_DIR, unique_filename)
    update_time = time.time()
    with open(filepath, 'wb') as f:
        for chunk in FileStream.yield_tar(_collect_data(start_date, end_date)):
            f.write(chunk)

            if (time.time() - update_time) > 3:
                update_time = time.time()
                self.update_state(state='EXECUTING', meta={'size': _file_size(f)})
    return {
        'filename': filename,
        'filepath': filepath,
    }


def _collect_data(start, end):
    for dir in LOG_DIR:
        for file_data in _get_files(start, end, dir):
            yield file_data


def _get_files(start, end, d):
    for filepath in glob.glob(d):
        if os.path.isdir(filepath):
            continue

        filename = os.path.basename(filepath)
        stat = os.stat(filepath)
        modified_time = datetime.datetime.fromtimestamp(stat.st_mtime)

        if modified_time.date() >= start and \
                modified_time.date() <= end:
            yield (filename, filepath, stat)
            continue

        creation_time = _get_creation_time(filepath, stat)
        if not creation_time:
            continue
        # log rotated file will have modified_time older then creation_time
        if modified_time < creation_time:
            continue
        if modified_time.date() >= start and \
                creation_time.date() <= end:
            yield (filename, filepath, stat)


def _get_creation_time(fp, st):
    ct = subprocess.check_output(
        'fs=$(df "%s"  | tail -1 | awk \'{print $1}\');' % fp +
        'echo $(sudo debugfs -R \'stat <\'"%s"\'>\' "${fs}" 2>/dev/null | grep -oP \'crtime.*--\\s*\\K.*\')' % st.st_ino,
        shell=True, universal_newlines=True)
    ct = ct.strip()
    if not ct:
        return
    return datetime.datetime.strptime(ct, '%a %b %d %H:%M:%S %Y')


def _file_size(f):
    size = os.path.getsize(f.name)
    return _sizeof_fmt(size)


def _sizeof_fmt(num):
    for unit in ('B', 'KB', 'MB', 'GB', 'TB'):
        if abs(num) < 1024.0:
            return '%3.1f %s' % (num, unit)
        num /= 1024.0
    return '%3.1f %s' % (num, 'PB')


class FileStream:
    def __init__(self):
        self.buffer = io.BytesIO()
        self.offset = 0

    def write(self, s):
        self.buffer.write(s)
        self.offset += len(s)

    def tell(self):
        return self.offset

    def close(self):
        self.buffer.close()

    def pop(self):
        s = self.buffer.getvalue()
        self.buffer.close()
        self.buffer = io.BytesIO()
        return s

    @classmethod
    def yield_tar(cls, files):
        stream = FileStream()
        tar = tarfile.open(mode='w|gz', fileobj=stream, bufsize=tarfile.BLOCKSIZE)

        for filename, filepath, stat in files:
            tar_info = tarfile.TarInfo(filepath[1:])
            fsize = int(stat.st_size)

            tar_info.size = fsize
            tar_info.mtime = stat.st_mtime
            tar.addfile(tar_info)
            yield stream.pop()

            blocks, remainder = divmod(tar_info.size, tarfile.BLOCKSIZE)

            process = subprocess.Popen(
                'sudo dd if="%s" bs=%s count=%s 2>/dev/null' % (filepath, tarfile.BLOCKSIZE, blocks + (1 if remainder > 0 else 0)),
                shell=True,
                stdout=subprocess.PIPE)

            while True:
                data = process.stdout.read(tarfile.BLOCKSIZE)
                if not data:
                    break

                if fsize < len(data):
                    data = data[:fsize]

                fsize = fsize - len(data)

                tar.fileobj.write(data)
                yield stream.pop()

                if fsize <= 0:
                    break

            if remainder > 0:
                tar.fileobj.write(tarfile.NUL * (tarfile.BLOCKSIZE - remainder))
                yield stream.pop()
                blocks += 1
            tar.offset += blocks * tarfile.BLOCKSIZE

        tar.close()
        yield stream.pop()
