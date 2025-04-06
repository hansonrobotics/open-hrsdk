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

set -e
BASEDIR=$(dirname $(readlink -f ${BASH_SOURCE[0]}))
source $BASEDIR/../common.sh

show_help() {
cat << EOF

Generate the batch scripts for docker images management.

Usage: $0 --region region digest_file
  --region
    AWS region name
  digest_file
    the digest file

Example:
    $0 --region ap-east-1 ./digests/v0.9b

EOF
}

while [[ $# > 0 ]];
do
    case "$1" in
        --region)
            region=$2
            shift
            shift
            ;;
        *)
            if [[ -z $digest_file ]]; then
                digest_file=$1
                shift
            else
                error "Unknown argument $1"
                show_help
                exit 1
            fi
            ;;
    esac
done

if [[ -z $region ]]; then
    error "--region was missing"
    show_help
    exit 1
fi
if [[ -z $digest_file ]]; then
    error "digest_file was missing"
    show_help
    exit 1
fi

if [[ ! -f $digest_file ]]; then
    error "digest_file was not found"
    exit 1
fi

ecr_prefix=222132024866.dkr.ecr.${region}.amazonaws.com/
tag=latest

info "tag commands"
cat $digest_file | awk -v ecr_prefix=$ecr_prefix -v tag=$tag '/hansonrobotics/ {print "docker tag " ecr_prefix $1 "@" $3 " " ecr_prefix $1 ":" tag }' | tee /tmp/tag.sh
info "push commands"
cat $digest_file | awk -v ecr_prefix=$ecr_prefix -v tag=$tag '/hansonrobotics/ {print "docker push " ecr_prefix $1 ":" tag }' | tee /tmp/push.sh

info "saved batch tag commands to /tmp/tag.sh /tmp/push.sh"
