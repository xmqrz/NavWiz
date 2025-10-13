#!/bin/sh
# postinst script for @(Package)
#
# see: dh_installdeb(1)

set -eu

USER=navwiz
GROUP=navwiz
HOME=/var/lib/navwiz
VENV="@(InstallationPrefix)"
IS_CHROOT=$(if [ "$(stat -c %d:%i /)" != "$(stat -c %d:%i /proc/1/root/.)" ]; then echo "true"; else echo "false"; fi)
IS_SUBIQUITY=$(if [ "@(Distribution)" != "trusty" ] && [ "@(Distribution)" != "bionic" ] && [ "$IS_CHROOT" = true ]; then echo "true"; else echo "false"; fi)
@{ python = 'python2.7' if Distribution in ['trusty', 'bionic'] else 'python3' }@

user_install() {
    # create group
    if ! getent group $GROUP >/dev/null; then
        addgroup --system $GROUP
    fi

    # create user if it isn't already there
    if ! getent passwd $USER >/dev/null; then
        adduser --system --ingroup $GROUP --home $HOME \
                --no-create-home --gecos "NavWiz" \
                --disabled-login $USER
    fi

    # update files' owner and permissions
    chown $USER:$GROUP $HOME
    find $HOME -mindepth 1 -maxdepth 1 \
       -path "$HOME/.navwiz_plugin" -prune \
       -o -exec chown -R $USER:$GROUP {} +
    if [ -e $HOME/.navwiz_plugin ]; then
        chown $USER:$GROUP $HOME/.navwiz_plugin
    fi
    chmod 0700 $HOME/.ssh
    chmod 0600 $HOME/.ssh/config
    chmod 0400 $HOME/.ssh/dfhub_rsa
    chmod 0400 /etc/sudoers.d/navwiz

    # add group-write permissions
    chmod 0775 $HOME/media
    chmod -R g+w $HOME/media

    # rosdep init and update
    #rosdep init || true
    #sudo -H -u $USER rosdep update
}

usbmount_install() {
    # allow mounting and unmounting of NTFS filesystems
    dpkg-divert --add --package @(Package) --rename /etc/usbmount/usbmount.conf
    sed -r 's/^FILESYSTEMS=.*$/FILESYSTEMS="vfat ext2 ext3 ext4 hfsplus ntfs ntfs-3g fuseblk"/' \
        /etc/usbmount/usbmount.conf.distrib > /etc/usbmount/usbmount.conf
}

chrony_install() {
    local NTP_ENABLED=false

    if systemctl is-enabled chrony >/dev/null 2>&1 || systemctl is-enabled systemd-timesyncd >/dev/null 2>&1; then
        NTP_ENABLED=true
    fi

    set +e
    systemctl disable --now systemd-timesyncd
    systemctl mask systemd-timesyncd || ln -s /dev/null /etc/systemd/system/systemd-timesyncd.service
    set -e

    if [ ! -e /etc/chrony/chrony.conf.distrib ]; then
        dpkg-divert --add --package @(Package) --rename /etc/chrony/chrony.conf
        sed -r 's/^(makestep .*)$/#\1/' /etc/chrony/chrony.conf.distrib > /etc/chrony/chrony.conf
        sed -i '$a\
\
allow 10/8\
allow 172.16/12\
allow 192.168/16\
local stratum 10\
leapsecmode slew\
maxslewrate 10000\
initstepslew 1.0 ntp.ubuntu.com' \
            /etc/chrony/chrony.conf
    fi

    set +e
    if $NTP_ENABLED; then
        systemctl enable --now chrony
    else
        systemctl disable --now chrony
    fi
    set -e
}

mysql_install() {
    if $IS_SUBIQUITY; then
        /usr/sbin/mysqld &
        sleep 5
    else
        service mysql start || true
        sleep 5
        service mysql status | grep -q "running\|Uptime"
    fi

    rm -f /etc/mysql/conf.d/agv05.cnf
    cp $VENV/share/agv05_webserver/mysql/agv05.cnf /etc/mysql/conf.d/agv05.cnf
    grep -q '^max_allowed_packet' /etc/mysql/mysql.conf.d/mysqld.cnf && \
        sed -i 's/^max_allowed_packet/# max_allowed_packet/' /etc/mysql/mysql.conf.d/mysqld.cnf || true

    set +e
    echo "CREATE DATABASE agv05 COLLATE utf8_unicode_ci;" | mysql
    echo "CREATE DATABASE agv05x COLLATE utf8_unicode_ci;" | mysql
    echo "CREATE USER 'agv05'@@'localhost' IDENTIFIED BY 'agv05';" | mysql
    echo "GRANT ALL PRIVILEGES ON \`agv05%\`.* TO 'agv05'@@'localhost';" | mysql
    mysql_tzinfo_to_sql /usr/share/zoneinfo | sed -e "s/Local time zone must be set--see zic manual page/local/" | mysql mysql
    if ! $IS_SUBIQUITY; then
        service mysql restart
    fi
    set -e
}

django_install() {
    # create soft-links
    if [ ! -L $VENV/lib/@python/dist-packages/static ]; then
        ln -s $HOME/static $VENV/lib/@python/dist-packages/static
    fi
    if [ ! -L $VENV/lib/@python/dist-packages/media ]; then
        ln -s $HOME/media $VENV/lib/@python/dist-packages/media
    fi
    if [ ! -L $HOME/media/map_quality ]; then
        ln -s $HOME/.ros/map_quality $HOME/media/map_quality
    fi

    # activate the ROS workspace
    set +u
    _OLD_VIRTUAL_PATH="$PATH"
    . $VENV/setup.sh
    set -u
    export DJANGO_SETTINGS_MODULE=agv05_webserver.settings-prod

    # generate secret key
    if [ ! -e $HOME/.secret_key ]; then
        echo "SECRET_KEY='$(django-admin generate_secret_key)'" > $HOME/.secret_key
        chown $USER:$GROUP $HOME/.secret_key
        chmod 0400 $HOME/.secret_key
    fi

    export TRACKLESS=0
    django-admin createcachetable
    django-admin migrate
    if django-admin dumpdata auth.user | grep -vq admin; then
        django-admin loaddata --app system default_users
    fi
    django-admin generate_panel_token

    export TRACKLESS=1
    django-admin createcachetable
    django-admin migrate
    if django-admin dumpdata auth.user | grep -vq admin; then
        django-admin loaddata --app system default_users
    fi
    django-admin generate_panel_token

    # collect staticfiles
    django-admin collectstatic --no-input --clear

    set +u
    # clear variables from ROS workspace
    export PATH="$_OLD_VIRTUAL_PATH"
    unset CMAKE_PREFIX_PATH
    unset CPATH
    unset LD_LIBRARY_PATH
    unset PKG_CONFIG_PATH
    unset PYTHONPATH
    unset ROSLISP_PACKAGE_DIRECTORIES
    unset ROS_DISTRO
    unset ROS_ETC_DIR
    unset ROS_MASTER_URI
    unset ROS_PACKAGE_PATH
    unset ROS_ROOT
    # clear other variables
    unset DJANGO_SETTINGS_MODULE
    unset TRACKLESS
    set -u
}

user_install
usbmount_install
chrony_install
mysql_install
django_install

case "$1" in
    configure)
    ;;

    abort-upgrade|abort-remove|abort-deconfigure)
    ;;

    *)
        echo "postinst called with unknown argument \`$1'" >&2
        exit 1
    ;;
esac

set +u

#DEBHELPER#

exit 0

# vim: ts=4:sw=4
