#!/usr/bin/env bash

##
## Copyright (C) 2017-2025 Hanson Robotics
##
## This program is free software: you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation, either version 3 of the License, or
## (at your option) any later version.
##
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with this program.  If not, see <https://www.gnu.org/licenses/>.
##

package_hrsdk-tool() {
    local reponame=hrsdk_tool

    mkdir -p $BASEDIR/src
    rsync -r --delete \
        --exclude ".git" \
        --exclude "package" \
        --exclude "manage" \
        --exclude "docker" \
        --exclude "license" \
        $BASEDIR/../ $BASEDIR/src/$reponame

    pushd $BASEDIR/src/$reponame/launch >/dev/null && go build && popd

    get_version $1

    local name=head-hrsdk-tool
    local desc="Hanson Robot SDK Tool"
    local url="https://api.github.com/repos/hansonrobotics/$reponame/releases"

    local arch=$(uname -m)
    if [[ $arch == 'aarch64' ]] ;then
        arch='arm64'
    elif [[ $arch == 'x86_64' ]]; then
        arch='amd64'
    fi

    launcher=$BASEDIR/_launch-$(uname -s)-$(uname -m)
    if [[ ! -f $launcher ]]; then
        curl -L "https://github.com/docker/compose/releases/download/v2.6.0/docker-compose-$(uname -s)-$(uname -m)" -o $launcher
        chmod +x $launcher
    else
        cp $launcher $BASEDIR/src/_launch
    fi
    # Download the binary files for servo tools
    cd src
    curl -s https://api.github.com/repos/hansonrobotics/servo_experiments/releases/latest|grep browser_download_url|grep ping| cut -d : -f 2,3 | tr -d \" |wget -qi -
    curl -s https://api.github.com/repos/hansonrobotics/servo_experiments/releases/latest|grep browser_download_url|grep rw| cut -d : -f 2,3 | tr -d \" |wget -qi -
    chmod +x rw
    chmod +x ping
    cd -

    fpm -C "${BASEDIR}" -s dir -t deb -n "${name}" -v "${version#v}" --vendor "${VENDOR}" \
        --url "${url}" --description "${desc}" ${ms} --force \
        --deb-no-default-config-files \
        -p $BASEDIR/${name}_VERSION_ARCH.deb \
        -d "jq" \
        -d "openssl" \
        -d "wget" \
        src/hrsdk_tool/common.sh=${HR_PREFIX}/hrsdk/ \
        src/hrsdk_tool/config.sh=${HR_PREFIX}/hrsdk/ \
        src/hrsdk_tool/init.sh=${HR_PREFIX}/hrsdk/ \
        src/hrsdk_tool/pull.sh=${HR_PREFIX}/hrsdk/ \
        src/hrsdk_tool/push.sh=${HR_PREFIX}/hrsdk/ \
        src/hrsdk_tool/scripts/command_wrapper.sh=/usr/bin/hrsdk \
        src/hrsdk_tool/scripts/hrsdk-completion.bash=${HR_PREFIX}/hrsdk/scripts/ \
        src/hrsdk_tool/shutdown_sdk.sh=${HR_PREFIX}/hrsdk/ \
        src/hrsdk_tool/start_sdk.sh=${HR_PREFIX}/hrsdk/ \
        src/hrsdk_tool/upload.sh=${HR_PREFIX}/hrsdk/ \
        src/hrsdk_tool/user_config_template.sh=${HR_PREFIX}/hrsdk/ \
        src/hrsdk_tool/x_auth=${HR_PREFIX}/hrsdk/ \
        src/rw=${HR_PREFIX}/hrsdk/scripts/ \
        src/ping=${HR_PREFIX}/hrsdk/scripts/ \
        src/hrsdk_tool/LICENSE=${HR_PREFIX}/hrsdk/ \
        src/hrsdk_tool/launch/launch=${HR_PREFIX}/hrsdk/bin/ \
        src/_launch=${HR_PREFIX}/hrsdk/bin/

    latest=$(ls -1 -t head-hrsdk-tool_* | head -n1)
    ln -sfT $latest head-hrsdk-tool_latest_$arch.deb

    # clean up
    rm -r src
}

if [[ $(readlink -f ${BASH_SOURCE[0]}) == $(readlink -f $0) ]]; then
    BASEDIR=$(dirname $(readlink -f ${BASH_SOURCE[0]}))
    source $BASEDIR/common.sh
    source $BASEDIR/config.sh
    set -e

    package_hrsdk-tool $@
fi
