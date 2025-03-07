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

# common.sh
#
# This file contains the most common functions and variables
# DO NOT change it
#

COLOR_INFO='\033[32m'
COLOR_WARN='\033[33m'
COLOR_ERROR='\033[31m'
COLOR_RESET='\033[0m'
info() {
    printf "${COLOR_INFO}%s${COLOR_RESET} " "$@"; printf '\n'
}
warn() {
    printf "${COLOR_WARN}%s${COLOR_RESET} " "$@"; printf '\n'
}
error() {
    printf "${COLOR_ERROR}%s${COLOR_RESET} " "$@"; printf '\n'
}
fail() {
    printf "${COLOR_ERROR}%s${COLOR_RESET} " "$@"; printf '\n'  1>&2 && return 1
}
get_confirm_warn() {
    LC_ALL="en_US.UTF-8"
    local message="${1:-Are you sure?}"
    local answer
    printf "${COLOR_WARN}%s${COLOR_RESET}" "$message"
    read -r answer
    ! printf '%s\n' "$answer" | grep -Eq "$(locale yesexpr)"
    confirm=$?
}
get_confirm() {
    LC_ALL="en_US.UTF-8"
    local message="${1:-Are you sure?}"
    local answer
    printf "%s" "$message"
    read -r answer
    ! printf '%s\n' "$answer" | grep -Eq "$(locale yesexpr)"
    confirm=$?
}

parse_stop_args() {
    while [[ $# > 0 ]]; do
        case "$1" in
            --project)
                export HR_PROJECT=$2
                shift
                shift
                ;;
            -h|--help)
                show_stop_help
                exit 0
                ;;
            *)
                error "Unknown option $1"
                show_stop_help
                exit 1
                ;;
        esac
    done

    update_env # should run after the arg parsing
}

show_stop_help() {
cat << EOF

Usage: hrsdk stop [--project <project>]

    --project <project>
        specify the project of the services (default: hrsdk)

EOF
}

sdk_shutdown() {
    parse_stop_args $@
    if [[ ! -f $VERSION_FILE ]]; then
        generate_version_file ${VERSION_FILE}
    fi
    source $VERSION_FILE
    if [[ -f $LAUNCH_FILE ]]; then
        $LAUNCHER $LAUNCHER_ARGS down --remove-orphans
    else
        fail "ERROR: Launch file was not found"
    fi
    rm -f /tmp/pulseaudio.socket
    rmdir $LOCKDIR >/dev/null 2>&1 && info "Shutdown"
    if [[ -f ${VERSION_FILE} ]]; then
        rm ${VERSION_FILE}
    fi
}
login_ecr() {
    if [[ ! -z ${ECR_URL} ]]; then
        info "Logging in ECR region \"$AWS_REGION_NAME\""
        docker info >/dev/null 2>&1 || fail "Couldn't run docker. Please check if docker is running by \"docker info\""
        aws ecr get-login-password --region $AWS_REGION_NAME \
            | docker login --username AWS --password-stdin \
            ${ECR_URL} >/dev/null 2>&1
    else
        error "ECR_URL is not set"
    fi
}

_source_env_dir() {
    local sdk_env_dir=$1
    local envfile
    if [[ -d $sdk_env_dir ]]; then
        shopt -s nullglob
        for envfile in $sdk_env_dir/*; do
            if [[ $envfile =~ .*\.sh$ ]]; then
                source $envfile
            fi
        done
    fi
}

update_env() {
    if [[ -z $HRSDK_VERSION ]]; then
        fail "ERROR: SDK Version was not set. Please try to run \"hrsdk init\"."
    fi

    if [[ $USING_LOCAL_STORAGE == 1 ]]; then
        export CONFIG_NAME=
        export HR_CONFIG_STORAGE=$HR_LOCAL_STORAGE
        export VERSION_FILE=$LOCAL_STORAGE_PATH/versions/$HRSDK_VERSION
        export LAUNCH_FILE=$LOCAL_STORAGE_PATH/main.yaml
        export LAUNCH_DIRS=$LOCAL_STORAGE_PATH/services
        export CONFIG_PACKAGE_FILE=$LOCAL_STORAGE_PATH/package.json
        export HW_CONFIGS_DIR=$LOCAL_STORAGE_PATH/configs
        export SERVICE_CONFIG_FILE=$LOCAL_STORAGE_PATH/services.txt
        _source_env_dir $LOCAL_STORAGE_PATH/env
    else
        export LOCAL_STORAGE_PATH=/tmp/hrsdk_local_storage
        mkdir -p $LOCAL_STORAGE_PATH # a placeholder for local storage

        export HR_CONFIG_STORAGE=$HR_STORAGE/$CONFIG_NAME
        export VERSION_FILE=$HR_BOOTSTRAP_DIR/versions/$HRSDK_VERSION
        export LAUNCH_FILE=$HR_BOOTSTRAP_DIR/main.yaml
        export LAUNCH_DIRS=$HR_BOOTSTRAP_DIR/services
        export CONFIG_PACKAGE_FILE=$HR_BOOTSTRAP_DIR/package.json
        export HW_CONFIGS_DIR=$HR_BOOTSTRAP_DIR/configs
        export SERVICE_CONFIG_FILE=$HR_BOOTSTRAP_DIR/services.txt
        _source_env_dir $HR_BOOTSTRAP_DIR/env
    fi

    export LAUNCHER_ARGS="-p ${HR_PROJECT:-hrsdk} -f $LAUNCH_FILE"
    if [[ -z $REGISTRY_PREFIX || $REGISTRY_PREFIX == default ]]; then
        if [[ $AWS_REGION_NAME == cn-* ]]; then
            # China region starts with cn- such as cn-north-1
            export ECR_URL=356078892395.dkr.ecr.${AWS_REGION_NAME}.amazonaws.com.cn
        else
            export ECR_URL=222132024866.dkr.ecr.${AWS_REGION_NAME}.amazonaws.com
        fi
    else
        export ECR_URL=$REGISTRY_PREFIX
    fi
    export AWS_STORAGE_REGION=$AWS_REGION_NAME  # back compatitility
}

compare_version() {
    # supports version specification with >=, <=, >, <, ^, =
    # 0: pass, 1: failure
    local current_version=$1
    local version_spec=$2
    local target_version
    if [[ $current_version != [0-9]* ]]; then
        return 0
    fi
    if [[ $version_spec == \>=* ]]; then
        target_version=${version_spec:2}
        if dpkg --compare-versions $current_version ge $target_version; then
            return 0
        fi
    elif [[ $version_spec == \<=* ]]; then
        target_version=${version_spec:2}
        if dpkg --compare-versions $current_version le $target_version; then
            return 0
        fi
    elif [[ $version_spec == ^* ]]; then
        target_version=${version_spec:1}
        local major=$(echo $target_version | cut -d. -f1)
        local minor=$(echo $target_version | cut -d. -f2)
        local revision=$(echo $target_version | cut -d. -f3)
        local next_version
        if [[ $major == 0 ]]; then
            next_version="0.$((minor+1))"
        else
            next_version="$((major+1)).0"
        fi
        if [[ ! -z $revision ]]; then
            next_version="$next_version.0"
        fi
        if dpkg --compare-versions $current_version ge $target_version && \
            dpkg --compare-versions $current_version lt $next_version ; then
            return 0
        fi
    elif [[ $version_spec == \>* ]]; then
        target_version=${version_spec:1}
        if dpkg --compare-versions $current_version gt $target_version; then
            return 0
        fi
    elif [[ $version_spec == \<* ]]; then
        target_version=${version_spec:1}
        if dpkg --compare-versions $current_version lt $target_version; then
            return 0
        fi
    elif [[ $version_spec == \=* ]]; then
        target_version=${version_spec:1}
        if dpkg --compare-versions $current_version eq $target_version; then
            return 0
        fi
    fi
    return 1
}

check_version() {
    # check configs version
    if [[ -f $CONFIG_PACKAGE_FILE ]]; then
        info "Checking compatibility"
        local current_version=$(cat $CONFIG_PACKAGE_FILE | jq -r ".version")
        local version_spec="^0.2.0" # the config vesion requirement
        if [[ -z $current_version ]]; then
            fail "ERROR: current config version not found"
        elif [[ -z $version_spec ]]; then
            fail "ERROR: config version spec is incorrect"
        else
            compare_version $current_version $version_spec || \
                fail "The SDK config requires version \"${version_spec}\", but you have \"$current_version\" which is incompatible"
        fi

        # check the debian package version
        for package in $(cat $CONFIG_PACKAGE_FILE | jq -r '.deb_packages | keys[]'); do
            current_version=$(dpkg-query -W -f='${Version}\n' $package 2>/dev/null)
            version_spec=$(cat $CONFIG_PACKAGE_FILE | jq -r ".deb_packages | .\"${package}\"")
            if [[ -z $current_version ]]; then
                fail "ERROR: ($package) current package version not found"
            elif [[ -z $version_spec ]]; then
                fail "ERROR: ($package) package version spec is incorrect"
            else
                compare_version $current_version $version_spec || \
                    fail "The package \"$package\" requires version \"${version_spec}\", but you have \"$current_version\" which is incompatible."
            fi
        done

        # check sdk version
        local sdk_version_spec=$(cat $CONFIG_PACKAGE_FILE | jq -r ".sdk_version")
        if [[ $sdk_version_spec == 'null' ]]; then
            return
        fi
        local sdk_current_version=${HRSDK_VERSION#v}
        if [[ -z $sdk_current_version ]]; then
            fail "ERROR: current SDK version not found"
        elif [[ -z $sdk_version_spec ]]; then
            fail "ERROR: SDK version spec is incorrect"
        else
            compare_version $sdk_current_version $sdk_version_spec || \
                fail "The config \"$CONFIG_NAME\" requires SDK version \"${sdk_version_spec}\", but you have \"$sdk_current_version\" which is incompatible"
        fi
    else
        error "Can't verify the configs. Check the storage path or upgrade the configs by \"hrsdk pull --configs\""
        exit 1
    fi
}

get_images() {
    # print docker images by the image group
    local group=$1
    local group=$(jq -r ".images | .${group}[] " $CONFIG_PACKAGE_FILE)
    local images image _image
    for image in ${group[@]}; do
        # expand the variable if it starts with $
        _image=${image#$}
        if [[ $image != $_image ]]; then
            eval image=\$\{$_image\}
        fi
        images=(${images[@]} $image)
    done
    echo ${images[@]}
}

get_image_by_service() {
    # prints the image by the service name
    local json_file=${VERSION_FILE}.json
    if [[ -f ${json_file} ]]; then
        local service=$1
        local image=$(cat $json_file| jq -r ".services.\"${service}\".image")
        local tag=$(cat $json_file | jq -r ".services.\"${service}\".tag")
        if [[ $(uname -m) == 'aarch64' ]]; then
            image=${image//hansonrobotics/hansonrobotics/arm64v8}
        fi
        if [[ ${DEV_MODE} == 1 ]]; then
            image=${image//hansonrobotics/hansonroboticsdev}
        fi
        echo ${image[@]}:${tag[@]}
    else
        error "No json version file was found"
    fi
}

get_images_by_service_group() {
    local json_file=${VERSION_FILE}.json
    local group_name=$@
    local image images
    local services services_deps
    if [[ $group_name == full ]]; then
        services=$(cat $json_file | jq -r ".services|keys[]")
    else
        services=$(cat $json_file | jq -r '.service_groups["'"${group_name}"'"][]' 2>/dev/null)
        services_deps=$(cat $json_file | jq -r '.service_groups["'"${group_name}_deps"'"][]' 2>/dev/null || echo "")
        if [[ ! -z $services_deps ]]; then
            services+=( ${services_deps[@]} )
        fi
    fi
    for service in ${services[@]}; do
        image=$(get_image_by_service $service)
        if [[ $image == null:null ]]; then
            error "Service $service was not found" >&2
            continue
        fi
        image="${image/\$REGISTRY/$ECR_URL}"
        images+=( ${image} )
    done

    local unique_images=( $(printf '%s\n' "${images[@]}" | sort -u) )
    echo ${unique_images[@]}
}


get_config_schema() {
    if [[ -f $CONFIG_PACKAGE_FILE ]]; then
        local schema=$(jq -r ".schema" $CONFIG_PACKAGE_FILE)
        if [[ $schema == 'null' ]]; then
            if [[ -f $SERVICE_CONFIG_FILE ]]; then
                schema_number=0.6
            else
                schema_number=0.5
            fi
        else
            schema_number=$schema
        fi
    else
        schema_number=0
    fi
}

generate_version_file() {
    # generates the plain version file from json version file
    local json_file=${VERSION_FILE}.json
    if [[ ! -f $json_file ]]; then
        return
    fi
    local version_file=$1
    >$version_file
    get_config_schema
    local services _service _image _fail
    case $schema_number in
        1.0)
            services=$(cat $json_file | jq -r ".services|keys[]")
            for service in ${services[@]}; do
                _image=$(get_image_by_service $service)
                if [[ $_image == null:null ]]; then
                    error "Service $service was not found"
                    _fail=1
                    continue
                fi
                _service=${service//-/_} # replace - by _
                echo "export ${_service^^}_IMAGE=$_image" >>$version_file
            done
            ;;
        1.1)
            services=$(cat $json_file | jq -r ".services|keys[]")
            for service in ${services[@]}; do
                _image=$(get_image_by_service $service)
                if [[ $_image == null:null ]]; then
                    error "Service $service was not found"
                    _fail=1
                    continue
                fi
                _image="${_image/\$REGISTRY/$ECR_URL}"
                _service=${service//-/_} # replace - by _
                echo "export ${_service^^}_IMAGE=$_image" >>$version_file
            done
            ;;
        *)
            error "Unknown schema $schema_number"
            ;;
    esac
    if [[ $_fail == 1 ]]; then
        >$version_file
        error "Version file generation failed"
    fi
}

detect_head_by_servo(){
    local id=$($HRSDK_INSTALL_DIR/scripts/ping --from_id 200 --to_id 230 --find_any true)
    if [ $? != 0 ]; then
        return 0
    fi
    local robot_head=$(grep --include motors_settings.yaml "motor_id: $id" -r $HW_CONFIGS_DIR/heads |cut -d":" -f1 |xargs -I {} dirname {} |xargs -I {} basename {})
    if [ $? == 0 ]; then
        export ROBOT_NAME=$robot_head
    fi
}

update_tablet_robot(){
	if [[ ${ROBOT_BODY: -1} == "X" ]]; then
        echo "Detecting tablet on $ROBOT_BODY"
		local position=$($HRSDK_INSTALL_DIR/scripts/rw --id 83  --addr 56 --length 2)
		# These are hardcoded values for elbows
		if [[ $position -lt 1000 || ($position != ?(-)+([0-9]))  ]]
		then
			export ROBOT_BODY="${ROBOT_BODY%?}T"
		else
			export ROBOT_BODY="${ROBOT_BODY%?}"
		fi
	fi
}


detect_hw() {
    local serial_dev=/dev/serial/by-id/usb*

    if [[ ! -d $HW_CONFIGS_DIR ]]; then
        error "Configs directory was not found. Pull the configs by \"hrsdk pull\""
        exit 1
    fi
    info "Detecting hardware by serial port"
    for dev in $serial_dev; do
        [ -e "$dev" ] || continue
        if [[ $dev == *"Prolific_Technology"* ]]; then
            continue
        fi
        local hardware_file=$(find $HW_CONFIGS_DIR -name 'hardware.launch' -exec grep -il "$dev" {} \;)
        local found=$(echo -n $hardware_file |grep -c '^' || true)
        if [[ $found > 1 ]]; then
            error "Multiple assemblies found for device $dev"
            exit 1
        fi
        if [[ $found < 1 ]]; then
            continue
        fi
        if [[ -z $ROBOT_NAME ]]; then
            if [[ $hardware_file == *"/heads/"* ]]; then
                export ROBOT_NAME=$(basename $(dirname $hardware_file))
            fi
        fi
        if [[ -z $ROBOT_BODY ]]; then
            if [[ $hardware_file == *"/bodies/"* ]]; then
                export ROBOT_BODY=$(basename $(dirname $hardware_file))
            fi
        fi
    done
    if [[ -z $ROBOT_NAME ]]; then
        detect_head_by_servo
    fi
    if [[ -z $ROBOT_NAME ]]; then
        if [[ -z $DEFAULT_ROBOT_NAME ]]; then
            error "Robot name was not found"
            exit 1
        fi
        export ROBOT_NAME=$DEFAULT_ROBOT_NAME
        warn "Using default robot name: $ROBOT_NAME"
        if [[ $ENABLE_HARDWARE != 1 ]]; then
            export HW_DETECT_FAIL=1
        fi
    fi
    if [[ -z $ROBOT_BODY ]]; then
        if [[ -z $DEFAULT_ROBOT_BODY ]]; then
            error "Robot body was not found"
            exit 1
        fi
        export ROBOT_BODY=$DEFAULT_ROBOT_BODY
        warn "Using default robot body: $ROBOT_BODY"
        if [[ $ENABLE_HARDWARE != 1 ]]; then
            export HW_DETECT_FAIL=1
        fi
    fi
}


export HR_DIR=/hr/.hrsdk  # SDK home directory where saves the logs, speech etc
export HR_STORAGE=/hr/storage # SDK storage directory where saves the configs, blender rigs, etc
export HR_LOCAL_STORAGE=/hr/local/storage # Local storage
export USER_CONFIG_DIR=$HOME/.hrsdk # User config directory
export HR_BOOTSTRAP_DIR=$HOME/.hrsdk/bootstrap
export LOCKDIR=/tmp/hrsdk.lock
export DOCKER_RUNTIME=runc
export HW_DETECT_FAIL=0
export SYNC_RESET=0
export GOOGLE_SPEECH=0
export LAUNCHER=/opt/hansonrobotics/hrsdk/bin/launch

# host files
export BLENDER_RIG_FILE=/tmp/empty.blend
export HRSDK_INSTALL_DIR=/opt/hansonrobotics/hrsdk
