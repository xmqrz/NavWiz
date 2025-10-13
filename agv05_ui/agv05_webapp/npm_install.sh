#!/bin/sh

set -eu

cd $(dirname $0)

deps_md5_file=node_modules/deps.md5
srcs_md5_file=node_modules/srcs.md5

new_deps=$(md5sum package.json)
new_srcs=$(find -L src -type f -not -name '.*' -exec md5sum {} \;)

same_deps=false
if [ -e $deps_md5_file ]; then
    old_deps=$(cat $deps_md5_file)
    if [ "$old_deps" = "$new_deps" ]; then
        same_deps=true
    fi
fi

if [ -d build ] && [ -e $srcs_md5_file ]; then
    old_srcs=$(cat $srcs_md5_file)
    if [ "$old_srcs" = "$new_srcs" ] && $same_deps; then
        echo "No change in source files. Skipping build."
        exit 0
    fi
fi

mkdir -p node_modules

if $same_deps; then
    echo "No change in dependencies. Skip installing node modules."
    npm run build
else
    npm install .
    touch node_modules/CATKIN_IGNORE
fi

echo "$new_deps" > $deps_md5_file
echo "$new_srcs" > $srcs_md5_file

# vim: ts=4:sw=4
