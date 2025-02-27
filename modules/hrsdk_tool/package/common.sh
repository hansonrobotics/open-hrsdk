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

if ! hash ruby >/dev/null 2>&1; then
    echo "Installing ruby-full"
    sudo apt-get install ruby-full
fi

if ! hash fpm >/dev/null 2>&1; then
    sudo gem install fpm deb-s3
    #gem install --install-dir $HOME/.hrsdk/gem fpm deb-s3
fi

if ! hash chrpath >/dev/null 2>&1; then
    echo "Installing chrpath"
    sudo apt-get install chrpath
fi

if ! hash autoconf >/dev/null 2>&1; then
    echo "Installing autoconf"
    sudo apt-get install autoconf
fi

if ! hash jq >/dev/null 2>&1; then
    echo "Installing jq"
    sudo apt-get install jq
fi

if [[ ! -f /usr/local/go/bin/go ]]; then
    echo "Installing go"
    arch=$(uname -m)
    if [[ $arch == 'aarch64' ]] ;then
        arch='arm64'
    elif [[ $arch == 'x86_64' ]]; then
        arch='amd64'
    fi
    wget https://go.dev/dl/go1.14.2.$(uname -s|tr '[:upper:]' '[:lower:]')-$arch.tar.gz -O /tmp/go.tar.gz
    sudo tar -C /usr/local -xzf /tmp/go.tar.gz
fi

export PATH=/usr/local/go/bin:$PATH

COLOR_INFO='\033[32m'
COLOR_WARN='\033[33m'
COLOR_ERROR='\033[31m'
COLOR_RESET='\033[0m'
info() {
    printf "${COLOR_INFO}[INFO] ${1}${COLOR_RESET}\n" >&2
}
warn() {
    printf "${COLOR_WARN}[WARN] ${1}${COLOR_RESET}\n" >&2
}
error() {
    printf "${COLOR_ERROR}[ERROR] ${1}${COLOR_RESET}\n" >&2
}

source_ros() {
    if [[ -e /opt/ros/indigo/setup.bash ]]; then
        source /opt/ros/indigo/setup.bash
    else if [[ -e /opt/ros/kinetic/setup.bash ]]; then
            source /opt/ros/kinetic/setup.bash
        fi
    fi
}

add_control_scripts() {
    local root_dir=${1:-${PACKAGE_DIR}/control}
    local preinst="${root_dir}/preinst.sh"
    local postinst="${root_dir}/postinst.sh"
    local prerm="${root_dir}/prerm.sh"
    local postrm="${root_dir}/postrm.sh"

    local ms=""
    [[ -f ${preinst} ]] && ms="$ms --before-install ${preinst}"
    [[ -f ${postinst} ]] && ms="$ms --after-install ${postinst}"
    [[ -f ${prerm} ]] && ms="$ms --before-remove ${prerm}"
    [[ -f ${postrm} ]] && ms="$ms --after-remove ${postrm}"

    if [[ -z $ms ]]; then
        echo "Empty maintainer scripts"
        return 1
    fi
    echo $ms
}

create_postint_for_python_deps() {
local output=$1
if [[ -z $output ]]; then
    error "No output file is specified. \ncreate_postint_for_python_deps output deps [[deps2] deps3]"
    return 1
fi
shift
if (( $# >= 1 )); then
mkdir -p $(dirname ${output})
touch ${output}
cat << EOF >${output}
#!/usr/bin/env bash

hr cmd pip2_install "$@"
EOF
else
    error "No dependency is specified. \ncreate_postint_for_python_deps output deps [[deps2] deps3]"
    return 1
fi
}

create_postint_for_python3_deps() {
local output=$1
if [[ -z $output ]]; then
    error "No output file is specified. \ncreate_postint_for_python3_deps output deps [[deps2] deps3]"
    return 1
fi
shift
if (( $# >= 1 )); then
mkdir -p $(dirname ${output})
touch ${output}
cat << EOF >${output}
#!/usr/bin/env bash

hr cmd pip3_install "$@"
EOF
else
    error "No dependency is specified. \ncreate_postint_for_python3_deps output deps [[deps2] deps3]"
    return 1
fi
}

cleanup_ros_package_build() {
    # clean up
    pushd $1 >/dev/null
    rm -r src build_isolated devel_isolated .catkin_workspace install
    popd >/dev/null
}

get_version() {
    local date=$(date +%Y%m%d%H%M%S)
    local version_file=$BASEDIR/src/$reponame/version
    local tag=$(git describe --tags --candidates=0)
    if [[ -f $version_file ]]; then
        version=$(head -n 1 $version_file)
        # if 1 is present or the latest tag equals to version
        if [[ $1 != 1 && ${tag#v} != $version ]]; then
            version=${version}-${date}
        fi
    else
        version=$date
    fi
}

