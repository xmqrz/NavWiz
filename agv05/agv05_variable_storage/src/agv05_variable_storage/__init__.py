from __future__ import absolute_import
from __future__ import unicode_literals

from sqlalchemy import Column, DateTime, String, Text, create_engine, func
from sqlalchemy.exc import SQLAlchemyError
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
import os
import rospy
try:
    from urllib.parse import urlparse
    from urllib.request import url2pathname
except Exception:
    from urlparse import urlparse
    from urllib import url2pathname


class _Unspecified(object):
    pass


_unspecified = _Unspecified()


class VariableStorage(object):

    def __init__(self):
        storage_url = ''
        param_name = rospy.search_param('variable_storage_url')
        if param_name:
            storage_url = rospy.get_param(param_name)

        self.backend = None
        if FileStorageBackend.match_url(storage_url):
            self.backend = FileStorageBackend(rospy.get_name(), storage_url)
        elif MysqlStorageBackend.match_url(storage_url):
            self.backend = MysqlStorageBackend(rospy.get_name(), storage_url)

    def has_variable(self, name):
        return self.backend.has_variable(name)

    def get_variable(self, name, default=_unspecified):
        return self.backend.get_variable(name, default)

    def set_variable(self, name, value):
        return self.backend.set_variable(name, value)

    def delete_variable(self, name):
        return self.backend.delete_variable(name)


class VariableStorageBackend(object):
    protocol = 'INVALID PROTOCOL'

    def has_variable(self, name):
        raise NotImplementedError()

    def get_variable(self, name, default=_unspecified):
        raise NotImplementedError()

    def set_variable(self, name, value):
        raise NotImplementedError()

    def delete_variable(self, name):
        raise NotImplementedError()

    @classmethod
    def match_url(cls, storage_url):
        return storage_url.startswith(cls.protocol)


"""
Implementation of FileStorageBackend class.
"""


class FileStorageBackend(VariableStorageBackend):
    protocol = 'file:///'

    def __init__(self, node_name, storage_url):
        self.initialized = False

        if not self.match_url(storage_url):
            rospy.logerr('[VariableStorage] File storage URL must start with "file:///". Storage backend is not used.')
            self.initialized = False
        else:
            self.folder_path = os.path.join(
                url2pathname(urlparse(storage_url).path),
                node_name.lstrip(os.path.sep))
            self.initialized = True

    def has_variable(self, name):
        if not self.initialized:
            return False

        file_path = os.path.join(self.folder_path, name)
        return os.path.isfile(file_path)

    def get_variable(self, name, default=_unspecified):
        if not self.initialized:
            if default != _unspecified:
                return default
            else:
                raise KeyError(name)

        file_path = os.path.join(self.folder_path, name)
        if os.path.isfile(file_path):
            try:
                with open(file_path, 'r') as f:
                    value = f.read()
                    return value
            except Exception as ex:
                rospy.logerr('[VariableStroage] Filesystem error: %s', ex)

        if default != _unspecified:
            return default
        else:
            raise KeyError(name)

    def set_variable(self, name, value):
        if not self.initialized:
            return

        try:
            os.makedirs(self.folder_path)
        except OSError as ex:
            if not os.path.isdir(self.folder_path):
                rospy.logerr('[VariableStroage] Filesystem error: %s', ex)
                return

        file_path = os.path.join(self.folder_path, name)
        try:
            with open(file_path, 'w') as f:
                f.write(value)
        except Exception as ex:
            rospy.logerr('[VariableStorage] Filesystem error: %s', ex)

    def delete_variable(self, name):
        if not self.initialized:
            return

        file_path = os.path.join(self.folder_path, name)
        try:
            os.remove(file_path)
        except Exception as ex:
            rospy.logerr('[VariableStorage] Filesystem error: %s', ex)


"""
Implementation of MysqlStorageBackend class.
"""

Base = declarative_base()


class MysqlStorageBackend(VariableStorageBackend):
    protocol = 'mysql://'

    def __init__(self, node_name, storage_url):
        self.initialized = False

        if not self.match_url(storage_url):
            rospy.logerr('[VariableStorage] MySQL storage URL must start with "mysql://". Storage backend is not used.')
            self.initialized = False
        else:
            hash_start = storage_url.find('#')
            self.db_url = 'mysql+mysqldb://' + storage_url[8:hash_start]

            table_name = 'variable_storage'
            if hash_start != -1:
                table_name = storage_url[hash_start + 1:]

            self.Variable = type(str('Variable'), (Base,), {
                '__tablename__': table_name,
                'name': Column(String(127), primary_key=True, nullable=False),
                'value': Column(Text(4294000000), nullable=False),
                'created': Column(DateTime, nullable=False, default=func.now()),
                'modified': Column(DateTime, nullable=False, default=func.now(), onupdate=func.now()),
            })

            self.prefix = node_name + '/'
            self.initialized = True

    def has_variable(self, name):
        if not self.initialized:
            return False

        session = None
        try:
            engine = create_engine(self.db_url)
            session = sessionmaker(bind=engine)()

            count = session.query(self.Variable).filter_by(name=self.prefix + name).count()
            session.commit()

            return count > 0

        except SQLAlchemyError as ex:
            rospy.logerr('[VariableStorage] Database error: %s', str(ex))
            if session:
                session.rollback()

        finally:
            if session:
                session.close()

        return False

    def get_variable(self, name, default=_unspecified):
        if not self.initialized:
            if default != _unspecified:
                return default
            else:
                raise KeyError(name)

        session = None
        try:
            engine = create_engine(self.db_url)
            session = sessionmaker(bind=engine)()

            variable = session.query(self.Variable).filter_by(name=self.prefix + name).first()
            session.commit()

            if variable:
                return variable.value

        except SQLAlchemyError as ex:
            rospy.logerr('[VariableStorage] Database error: %s', str(ex))
            if session:
                session.rollback()

        finally:
            if session:
                session.close()

        if default != _unspecified:
            return default
        else:
            raise KeyError(name)

    def set_variable(self, name, value):
        if not self.initialized:
            return

        session = None
        try:
            engine = create_engine(self.db_url)
            Base.metadata.create_all(engine, checkfirst=True)
            session = sessionmaker(bind=engine)()

            variable = self.Variable()
            variable.name = self.prefix + name
            variable.value = value
            session.merge(variable)

            session.commit()

        except SQLAlchemyError as ex:
            rospy.logerr('[VariableStorage] Database error: %s', str(ex))
            if session:
                session.rollback()

        finally:
            if session:
                session.close()

    def delete_variable(self, name):
        if not self.initialized:
            return

        session = None
        try:
            engine = create_engine(self.db_url)
            session = sessionmaker(bind=engine)()

            session.query(self.Variable).filter_by(name=self.prefix + name).delete()
            session.commit()

        except SQLAlchemyError as ex:
            rospy.logerr('[VariableStorage] Database error: %s', str(ex))
            if session:
                session.rollback()

        finally:
            if session:
                session.close()
