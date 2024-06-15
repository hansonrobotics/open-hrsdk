##
## Copyright (C) 2017-2024 Hanson Robotics
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
env() {
    export HR_PREFIX=/opt/hansonrobotics
    export HR_BIN_PREFIX=$HR_PREFIX/bin
    export HRTOOL_PREFIX=${HR_PREFIX}/hrtool
    export HR_ROS_PREFIX=${HR_PREFIX}/ros
    export HR_TOOLS_PREFIX=$HR_PREFIX/tools
    export HR_DATA_PREFIX=$HR_PREFIX/data
    export VOICE_CACHE_DIR=$HOME/.hr/tts/voice
    export URL_PREFIX=https://github.com/hansonrobotics
    export GITHUB_STORAGE_URL=https://raw.githubusercontent.com/hansonrobotics/binary_dependency/master
    export GITHUB_STORAGE_URL2=https://$GITHUB_TOKEN@raw.githubusercontent.com/hansonrobotics/binary_dependency2/master
    export VENDOR="Hanson Robotics"
    export PYTHON_PKG_PREFIX=$HR_PREFIX/py2env/lib/python2.7/dist-packages
    export PYTHON3_PKG_PREFIX=$HR_PREFIX/py3env/lib/python3.6/dist-packages
    export ROS_PYTHON_PKG_PREFIX=$HR_ROS_PREFIX/lib/python2.7/dist-packages
}

install_deps() {
    if ! hash gem >/dev/null 2>&1; then
        echo "Installing ruby-full"
        sudo apt-get install ruby-full
    fi

    if ! hash fpm >/dev/null 2>&1; then
        echo "Installing fpm"
        sudo gem install fpm
        sudo gem install deb-s3
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
        wget https://dl.google.com/go/go1.14.2.linux-amd64.tar.gz -O /tmp/go1.14.2.linux-amd64.tar.gz
        sudo tar -C /usr/local -xzf /tmp/go1.14.2.linux-amd64.tar.gz
    fi

    export PATH=/usr/local/go/bin:$PATH
}

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
    local rosdistro=$(find_rosdistro)
    if [[ ! -z $rosdistro ]]; then
        info "ROS distribution $rosdistro"
        source /opt/ros/$rosdistro/setup.bash
    else
        error "ROS distribution is not found"
        return 1
    fi
}

find_rosdistro() {
    local ros_rosdistros=(noetic melodic kinetic indigo)
    for rosdistro in ${ros_rosdistros[@]}; do
        if [[ -e /opt/ros/$rosdistro/setup.bash ]]; then
            echo "$rosdistro"
        fi
    done
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

env
install_deps
