#!/bin/sh
# postrm script for @(Package)
#
# see: dh_installdeb(1)

set -eu

USER=navwiz
GROUP=navwiz
HOME=/var/lib/navwiz
VENV="@(InstallationPrefix)"
@{ python = 'python2.7' if Distribution in ['trusty', 'bionic'] else 'python3' }@

user_purge() {
    # delete user
    if getent passwd $USER >/dev/null; then
        deluser --system $USER
    fi

    # remove home folder
    rm -rf $HOME

    # delete group
    if getent group $GROUP >/dev/null; then
        delgroup --system $GROUP
    fi
}

chrony_purge() {
    # revert modification of chrony.conf
    if [ -f /etc/chrony/chrony.conf.distrib ]; then
        rm -f /etc/chrony/chrony.conf
        dpkg-divert --remove --package @(Package) --rename /etc/chrony/chrony.conf || true
    fi
}

mysql_purge() {
    set +e
    rm -f /etc/mysql/conf.d/agv05.cnf
    echo "DROP DATABASE agv05;" | mysql
    echo "DROP DATABASE agv05x;" | mysql
    service mysql restart
    set -e
}

django_purge() {
    # remove soft-links
    if [ -L $VENV/lib/@python/dist-packages/static ]; then
        rm $VENV/lib/@python/dist-packages/static
    fi
    if [ -L $VENV/lib/@python/dist-packages/media ]; then
        rm $VENV/lib/@python/dist-packages/media
    fi
    if [ -L $HOME/media/map_quality ]; then
        rm $HOME/media/map_quality
    fi

    # remove secret key
    if [ -f $HOME/.secret_key ]; then
        rm $HOME/.secret_key
    fi

    # remove panel token
    rm -f $VENV/share/agv05_webapp/.token.js
    rm -f $VENV/share/agv05_webapp/.token-x.js
}

usbmount_remove() {
    # revert modification of usbmount.conf
    if [ -f /etc/usbmount/usbmount.conf.distrib ]; then
        rm -f /etc/usbmount/usbmount.conf
        dpkg-divert --remove --package @(Package) --rename /etc/usbmount/usbmount.conf || true
    fi
}


case "$1" in
    purge)
        django_purge
        mysql_purge
        chrony_purge
        user_purge
    ;;

    remove)
        usbmount_remove
    ;;

    upgrade|failed-upgrade|abort-install|abort-upgrade|disappear)
    ;;

    *)
        echo "postrm called with unknown argument \`$1'" >&2
        exit 1
    ;;
esac

set +u

#DEBHELPER#

exit 0

# vim: ts=4:sw=4
