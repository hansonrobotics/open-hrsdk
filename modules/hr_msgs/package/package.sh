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
package() {
    local reponame=hr_msgs

    mkdir -p $BASEDIR/src
    rsync -r --delete \
        --exclude ".git" \
        --exclude "package" \
        $BASEDIR/../ $BASEDIR/src/$reponame

    get_version $1
    source_ros
    catkin_make_isolated --directory $BASEDIR --install --install-space $BASEDIR/install -DCMAKE_BUILD_TYPE=Release

    local name
    local desc="ROS message definition for Hanson Robotics"
    local url="https://api.github.com/repos/hansonrobotics/$reponame/releases"

    local pythonpath
    local rosdistro=$(find_rosdistro)
    if [[ $rosdistro == noetic ]]; then
        pythonpath=install/lib/python3/
    else
        pythonpath=install/lib/python2.7/
    fi

    for dist in kinetic melodic ; do
        name=head-$dist-hr_msgs
        ROS_PREFIX=/opt/ros/$dist

        fpm -C "${BASEDIR}" -s dir -t deb -n "${name}" -v "${version#v}" --vendor "${VENDOR}" \
            --url "${url}" --description "${desc}" ${ms} --force \
            --deb-no-default-config-files \
            -p $BASEDIR/${name}_VERSION_ARCH.deb \
            install/include=${ROS_PREFIX}/ \
            install/share=${ROS_PREFIX}/ \
            install/lib/pkgconfig/=${ROS_PREFIX}/lib/pkgconfig/ \
            $pythonpath=${ROS_PREFIX}/lib/python2.7/
    done

    for dist in noetic; do
        name=head-$dist-hr_msgs
        ROS_PREFIX=/opt/ros/$dist

        fpm -C "${BASEDIR}" -s dir -t deb -n "${name}" -v "${version#v}" --vendor "${VENDOR}" \
            --url "${url}" --description "${desc}" ${ms} --force \
            --deb-no-default-config-files \
            -p $BASEDIR/${name}_VERSION_ARCH.deb \
            install/include=${ROS_PREFIX}/ \
            install/share=${ROS_PREFIX}/ \
            install/lib/pkgconfig/=${ROS_PREFIX}/lib/pkgconfig/ \
            $pythonpath=${ROS_PREFIX}/lib/python3/
    done

    cleanup_ros_package_build $BASEDIR
}

if [[ $(readlink -f ${BASH_SOURCE[0]}) == $(readlink -f $0) ]]; then
    BASEDIR=$(dirname $(readlink -f ${BASH_SOURCE[0]}))
    source $BASEDIR/common.sh
    set -e

    package $1
fi
