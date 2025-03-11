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

_detect_chest_camera() {
    shopt -s nullglob
    for dev in /dev/video*; do
        if [[ -c $dev ]]; then
            for link in $(udevadm info -q symlink $dev); do
                if [[ $link =~ .*by-id.* && ! $link =~ .*RealSense.* ]]; then
                    local confirm
                    local camera=/dev/${link}
                    local ID_MODEL=$(udevadm info -q all $camera |grep ID_MODEL=|cut -d= -f2)
                    echo "Detected web camera \"$camera\", model: \"$ID_MODEL\""
                    get_confirm "Set it as the chest camera? [y/n] "
                    if [[ ${confirm} -eq 1 ]]; then
                        chest_camera=${camera}
                        return
                    fi
                fi
            done
        fi
    done
}

make_config() {
    mkdir -p $USER_CONFIG_DIR

    if [[ -f $config_file ]]; then
        # read current config.sh
        local CURRENT_AWS_ACCESS_KEY_ID=$(cat $config_file |grep -e "^export AWS_ACCESS_KEY_ID="|tail -n1|cut -d= -f2)
        local CURRENT_AWS_SECRET_ACCESS_KEY=$(cat $config_file |grep -e "^export AWS_SECRET_ACCESS_KEY="|tail -n1|cut -d= -f2)
        local CURRENT_AWS_REGION_NAME=$(cat $config_file |grep -e "^export AWS_REGION_NAME="|tail -n1|cut -d= -f2)
        local CURRENT_REGISTRY_PREFIX=$(cat $config_file |grep -e "^export REGISTRY_PREFIX="|tail -n1|cut -d= -f2)
        local CURRENT_CONFIG_NAME=$(cat $config_file |grep -e "^export CONFIG_NAME="|tail -n1|cut -d= -f2)
        local CURRENT_HRSDK_VERSION=$(cat $config_file |grep -e "^export HRSDK_VERSION="|tail -n1|cut -d= -f2)
        local CURRENT_ENVIRONMENT=$(cat $config_file |grep -e "^export ENVIRONMENT="|tail -n1|cut -d= -f2)
        if [[ -z $CURRENT_REGISTRY_PREFIX ]]; then
            CURRENT_REGISTRY_PREFIX=default
        fi
        if [[ -z $CURRENT_ENVIRONMENT ]]; then
            CURRENT_ENVIRONMENT=default
        fi
    fi
    if [[ ! -z $CURRENT_AWS_ACCESS_KEY_ID ]]; then
        local CURRENT_AWS_ACCESS_KEY_ID_MASK=" [$(echo $CURRENT_AWS_ACCESS_KEY_ID | sed "s/.*\(.\{4\}\)$/****************\1/")]"
    fi
    if [[ ! -z $CURRENT_AWS_SECRET_ACCESS_KEY ]]; then
        local CURRENT_AWS_SECRET_ACCESS_KEY_MASK=" [$(echo $CURRENT_AWS_SECRET_ACCESS_KEY | sed "s/.*\(.\{4\}\)$/****************\1/")]"
    fi
    if [[ ! -z $CURRENT_AWS_REGION_NAME ]]; then
        local CURRENT_AWS_REGION_NAME_MASK=" [$CURRENT_AWS_REGION_NAME]"
    fi
    if [[ ! -z $CURRENT_REGISTRY_PREFIX ]]; then
        local CURRENT_REGISTRY_PREFIX_MASK=" [$CURRENT_REGISTRY_PREFIX]"
    fi
    if [[ ! -z $CURRENT_CONFIG_NAME ]]; then
        local CURRENT_CONFIG_NAME_MASK=" [$CURRENT_CONFIG_NAME]"
    fi
    if [[ ! -z $CURRENT_HRSDK_VERSION ]]; then
        local CURRENT_HRSDK_VERSION_MASK=" [$CURRENT_HRSDK_VERSION]"
    fi
    if [[ ! -z $CURRENT_ENVIRONMENT ]]; then
        local CURRENT_ENVIRONMENT_MASK=" [$CURRENT_ENVIRONMENT]"
    fi

	if [[ $yes == 1 ]]; then
		if [[ ! -z $CURRENT_AWS_ACCESS_KEY_ID ]]; then
			AWS_ACCESS_KEY_ID=$CURRENT_AWS_ACCESS_KEY_ID
		else
			error "AWS Access Key ID can't be empty"
			exit 1
		fi

        if [[ ! -z $CURRENT_AWS_SECRET_ACCESS_KEY ]]; then
            AWS_SECRET_ACCESS_KEY=$CURRENT_AWS_SECRET_ACCESS_KEY
        else
            error "AWS Secret Access Key can't be empty"
            exit 1
        fi

        if [[ ! -z $CURRENT_AWS_REGION_NAME ]]; then
            AWS_REGION_NAME=$CURRENT_AWS_REGION_NAME
        else
            error "Default region name can't be empty"
            exit 1
        fi

        if [[ ! -z $CURRENT_CONFIG_NAME ]]; then
            CONFIG_NAME=$CURRENT_CONFIG_NAME
        else
            error "Config name can't be empty"
            exit 1
        fi

        if [[ ! -z $CURRENT_HRSDK_VERSION ]]; then
            HRSDK_VERSION=$CURRENT_HRSDK_VERSION
        else
            error "Version can't be empty"
            exit 1
        fi

        if [[ ! -z $CURRENT_ENVIRONMENT ]]; then
            ENVIRONMENT=$CURRENT_ENVIRONMENT
        else
            error "Environment can't be empty"
            exit 1
        fi
    else
        # get user input
        echo -n "AWS Access Key ID$CURRENT_AWS_ACCESS_KEY_ID_MASK: "
        read AWS_ACCESS_KEY_ID
        if [[ -z $AWS_ACCESS_KEY_ID ]]; then
            if [[ ! -z $CURRENT_AWS_ACCESS_KEY_ID ]]; then
                AWS_ACCESS_KEY_ID=$CURRENT_AWS_ACCESS_KEY_ID
            else
                error "AWS Access Key ID can't be empty"
                exit 1
            fi
        fi

        echo -n "AWS Secret Access Key$CURRENT_AWS_SECRET_ACCESS_KEY_MASK: "
        read AWS_SECRET_ACCESS_KEY
        if [[ -z $AWS_SECRET_ACCESS_KEY ]]; then
            if [[ ! -z $CURRENT_AWS_SECRET_ACCESS_KEY ]]; then
                AWS_SECRET_ACCESS_KEY=$CURRENT_AWS_SECRET_ACCESS_KEY
            else
                error "AWS Secret Access Key can't be empty"
                exit 1
            fi
        fi

        echo -n "Default region name$CURRENT_AWS_REGION_NAME_MASK: "
        read AWS_REGION_NAME
        if [[ -z $AWS_REGION_NAME ]]; then
            if [[ ! -z $CURRENT_AWS_REGION_NAME ]]; then
                AWS_REGION_NAME=$CURRENT_AWS_REGION_NAME
            else
                error "Default region name can't be empty"
                exit 1
            fi
        fi

        echo -n "Registry prefix$CURRENT_REGISTRY_PREFIX_MASK: "
        read REGISTRY_PREFIX
        if [[ -z $REGISTRY_PREFIX ]]; then
            if [[ ! -z $CURRENT_REGISTRY_PREFIX ]]; then
                REGISTRY_PREFIX=$CURRENT_REGISTRY_PREFIX
            fi
        fi

        echo -n "Config name$CURRENT_CONFIG_NAME_MASK: "
        read CONFIG_NAME
        if [[ -z $CONFIG_NAME ]]; then
            if [[ ! -z $CURRENT_CONFIG_NAME ]]; then
                CONFIG_NAME=$CURRENT_CONFIG_NAME
            else
                error "Config name can't be empty"
                exit 1
            fi
        fi

        echo -n "HRSDK Version$CURRENT_HRSDK_VERSION_MASK: "
        read HRSDK_VERSION
        if [[ -z $HRSDK_VERSION ]]; then
            if [[ ! -z $CURRENT_HRSDK_VERSION ]]; then
                HRSDK_VERSION=$CURRENT_HRSDK_VERSION
            else
                error "Version can't be empty"
                exit 1
            fi
        fi

        echo -n "Environment$CURRENT_ENVIRONMENT_MASK: "
        read ENVIRONMENT
        if [[ -z $ENVIRONMENT ]]; then
            if [[ ! -z $CURRENT_ENVIRONMENT ]]; then
                ENVIRONMENT=$CURRENT_ENVIRONMENT
            else
                error "Environment can't be empty"
                exit 1
            fi
        fi
	fi

    export AWS_ACCESS_KEY_ID
    export AWS_SECRET_ACCESS_KEY
    export AWS_REGION_NAME
    export REGISTRY_PREFIX
    export CONFIG_NAME
    export HRSDK_VERSION
    export ENVIRONMENT

    _detect_chest_camera

    # make the config template
    local template_file=/tmp/config.template
    #if [[ -f $config_file ]]; then
    #    cp $config_file $template_file
    #else
    #    cp $BASEDIR/user_config_template.sh $template_file
    #fi
    cp $BASEDIR/user_config_template.sh $template_file

    # check the required env vars and add them if they are missing
    for var in  \
        AWS_ACCESS_KEY_ID \
        AWS_SECRET_ACCESS_KEY \
        AWS_REGION_NAME \
        REGISTRY_PREFIX \
        CONFIG_NAME \
        HRSDK_VERSION \
        CHEST_CAMERA_DEVICE \
        ENVIRONMENT \
        ;
    do
        if ! grep -q -e "^export $var=" $template_file ; then
            echo "export $var=" >>$template_file
        fi
    done

    sed -i "s#export AWS_ACCESS_KEY_ID=.*#export AWS_ACCESS_KEY_ID=${AWS_ACCESS_KEY_ID}#" $template_file
    sed -i "s#export AWS_SECRET_ACCESS_KEY=.*#export AWS_SECRET_ACCESS_KEY=${AWS_SECRET_ACCESS_KEY}#" $template_file
    sed -i "s#export AWS_REGION_NAME=.*#export AWS_REGION_NAME=${AWS_REGION_NAME}#" $template_file
    sed -i "s#export REGISTRY_PREFIX=.*#export REGISTRY_PREFIX=${REGISTRY_PREFIX}#" $template_file
    sed -i "s#export CONFIG_NAME=.*#export CONFIG_NAME=${CONFIG_NAME}#" $template_file
    sed -i "s#export HRSDK_VERSION=.*#export HRSDK_VERSION=${HRSDK_VERSION}#" $template_file
    sed -i "s#export CHEST_CAMERA_DEVICE=.*#export CHEST_CAMERA_DEVICE=${chest_camera}#" $template_file
    sed -i "s#export ENVIRONMENT=.*#export ENVIRONMENT=${ENVIRONMENT}#" $template_file

    mv $template_file $config_file
    chmod 600 $config_file
}

show_help() {
cat << EOF

Usage: hrsdk init [-f] [-y]

    -f: force to re-initialize
    -y: yes to accept the existing values

EOF
}

if [[ $(readlink -f ${BASH_SOURCE[0]}) == $(readlink -f $0) ]]; then
    set -e
    BASEDIR=$(dirname $(readlink -f ${BASH_SOURCE[0]}))
    source $BASEDIR/common.sh
    if [[ -z $USER_CONFIG_DIR ]]; then
        error "user config directory is not configured"
        exit 1
    fi

    config_file=$USER_CONFIG_DIR/config.sh

	OPTIND=1  # Reset in case getopts has been used previously in the shell.
	while getopts "hyf" opt; do
		case "$opt" in
		h)
			show_help
			exit 0
			;;
		y)  yes=1
			;;
		f)  force=1
			;;
		esac
	done
	shift $((OPTIND-1))
	[ "${1:-}" = "--" ] && shift

    if [[ ! -f $config_file ]]; then
        make_config
    else
        if [[ $force == 1 ]]; then
            make_config
        else
            warn "SDK has already been initialzed. Run \"hrsdk init -f\" to re-initialize"
        fi
    fi
fi

