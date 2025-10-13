#!/bin/sh
# Setup the database for Django
#

set -eu

cd $(dirname $0)

mysql_install() {
    sudo service mysql start || true
    sleep 5
    sudo service mysql status | grep -q "running\|Uptime"

    sudo rm /etc/mysql/conf.d/agv05.cnf || true
    sudo cp ./debian_files/mysql/agv05.cnf /etc/mysql/conf.d/agv05.cnf

    set +e
    echo "CREATE DATABASE agv05 COLLATE utf8_unicode_ci;" | sudo mysql
    echo "CREATE DATABASE agv05x COLLATE utf8_unicode_ci;" | sudo mysql
    echo "CREATE USER 'agv05'@'localhost' IDENTIFIED BY 'agv05';" | sudo mysql
    echo "GRANT ALL PRIVILEGES ON \`agv05%\`.* TO 'agv05'@'localhost';" | sudo mysql
    mysql_tzinfo_to_sql /usr/share/zoneinfo | sed -e "s/Local time zone must be set--see zic manual page/local/" | sudo mysql mysql
    sudo service mysql restart
    set -e
}

django_install() {
    export DJANGO_SETTINGS_MODULE=agv05_webserver.settings

    export TRACKLESS=0
    django-admin createcachetable
    django-admin migrate
    if django-admin dumpdata auth.user | grep -vq admin; then
        django-admin loaddata --app system default_users
    fi

    export TRACKLESS=1
    django-admin createcachetable
    django-admin migrate
    if django-admin dumpdata auth.user | grep -vq admin; then
        django-admin loaddata --app system default_users
    fi

    unset DJANGO_SETTINGS_MODULE
    unset TRACKLESS
}


mysql_install
django_install

# vim: ts=4:sw=4
