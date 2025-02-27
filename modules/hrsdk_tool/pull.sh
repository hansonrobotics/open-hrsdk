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

# Pull the files from ECR and S3
#

show_help() {
cat << EOF

Usage: hrsdk pull [--reset] [--configs] [--images <environment>] [--models]
                  [--project <project>]

    --reset
        reset the storage
    --configs
        pull configs only
    --images <environment>
        pull the images for the service group
    --models
        pull models only
    --backup
        backup the images
    --no-bootstrap
        do not pull bootstrap files
    --project <project>
        specify the project of the services (default project: hrsdk)
    --dev
        dev mode

EOF
}

parse_args() {
    while [[ $# > 0 ]]; do
        case "$1" in
            --reset)
                get_confirm_warn "Are you sure to reset the configs [y/n] "
                if [[ $confirm == 1 ]]; then
                    info "Reset"
                    export SYNC_RESET=1
                else
                    info "Not reset"
                fi
                shift
                ;;
            --configs)
                export PULL_SDK_CONFIGS=1
                shift
                ;;
            --images)
                export PULL_SDK_IMAGES=1
                environment=$2
                if [[ ! -z $environment ]]; then
                    shift
                fi
                shift
                ;;
            --models)
                export PULL_SDK_MODELS=1
                shift
                ;;
            --backup)
                export PULL_BACKUP=1
                shift
                ;;
            --no-bootstrap)
                export NO_BOOTSTRAP_FILES=1
                shift
                ;;
            --project)
                export HR_PROJECT=$2
                shift
                shift
                ;;
            --dev)
                export DEV_MODE=1
                info "Run in dev mode"
                shift
                ;;
            -h|--help)
                show_help
                exit 0
                ;;
            *)
                error "Unknown option $1"
                show_help
                exit 1
                ;;
        esac
    done

    # default options
    if [[ $PULL_SDK_CONFIGS != 1 && \
            $PULL_SDK_IMAGES != 1 && \
            $PULL_SDK_MODELS != 1
        ]]; then
        export PULL_SDK_CONFIGS=1
        export PULL_SDK_IMAGES=1
        export PULL_SDK_MODELS=1
        environment=${ENVIRONMENT:-full}
    fi
}

_get_bootstrap_files() {
    info "Updating bootstrap files"
    if [[ $AWS_REGION_NAME == cn-* ]]; then
        export AWS_DEFAULT_REGION=cn-north-1
    else
        export AWS_DEFAULT_REGION=ap-east-1
    fi
    # get launch file
    aws s3 cp --only-show-errors \
        s3://dl.hrsdk.hansonrobotics.com/${CONFIG_NAME}/main.yaml \
        ${LAUNCH_FILE} || fail "ERROR: Couldn't get main launch file"

    # get package file
    aws s3 cp --only-show-errors \
        s3://dl.hrsdk.hansonrobotics.com/${CONFIG_NAME}/package.json \
        $HR_BOOTSTRAP_DIR/package.json || fail "ERROR: Couldn't get package file"

    local config_version=$(cat $HR_BOOTSTRAP_DIR/package.json | jq -r ".version")

    info "Config version $config_version"
    if compare_version $config_version "<0.2.2"; then
        # get services file
        aws s3 cp --only-show-errors \
            s3://dl.hrsdk.hansonrobotics.com/${CONFIG_NAME}/services.txt \
            $HR_BOOTSTRAP_DIR/services.txt || error "ERROR: Couldn't get service file"
    fi

    # get container version file (json format)
    aws s3 cp --only-show-errors \
        s3://dl.hrsdk.hansonrobotics.com/${CONFIG_NAME}/versions/${HRSDK_VERSION}.json \
        ${VERSION_FILE}.json || \
    aws s3 cp --only-show-errors \
        s3://dl.hrsdk.hansonrobotics.com/${CONFIG_NAME}/versions/${HRSDK_VERSION} \
        ${VERSION_FILE} || fail "ERROR: Couldn't get version info"

    # get env files
    aws s3 sync --only-show-errors --delete \
        s3://dl.hrsdk.hansonrobotics.com/${CONFIG_NAME}/env \
        $HR_BOOTSTRAP_DIR/env || true

    # check the compatibility for the SDK and configs
    check_version
}

_get_storage() {
    info "Updating storage"
    generate_version_file ${VERSION_FILE}
    source $VERSION_FILE
    export HR_CONFIG_STORAGE=$HR_STORAGE/$CONFIG_NAME
    if [[ -z $ROBOT_NAME || -z $ROBOT_BODY ]]; then
        detect_hw
    fi
    export LOCAL_STORAGE_PATH=/tmp/hrsdk_local_storage
    mkdir -p $LOCAL_STORAGE_PATH # a placeholder for local storage
    $LAUNCHER $LAUNCHER_ARGS run --rm storage
}

_get_service_data() {
    generate_version_file ${VERSION_FILE}
    source $VERSION_FILE
    get_config_schema
    info "Schema $schema_number"
    local gpu_services=()
    local services=()
    case "$schema_number" in
        1.1)
            # service groups moved to version json file
            local json_file=${VERSION_FILE}.json
            services=$(cat $json_file | jq -r ".service_groups|.services_require_data|.[]" 2>/dev/null) || services=()
            services=( $(printf '%s\n' "${services[@]}" | sort -u) )  # remove duplicates and join multiple lines
            gpu_services=$(cat $json_file | jq -r ".service_groups|.gpu_services|.[]" 2>/dev/null)
            gpu_services=( $(printf '%s\n' "${gpu_services[@]}" | sort -u) )  # remove duplicates and join multiple lines
            ;;
        *)
            fail "Config schema $schema_number is not supported"
            ;;
    esac
    if (( ${#services[@]} != 0 )); then
        info "Updating sdk data"

        NVIDIA_RUNTIME=$(docker info |grep Runtimes |grep nvidia|wc -l)
        if [[ $NVIDIA_RUNTIME != 1 ]]; then
            # remove the GPU sevices
            for target in "${gpu_services[@]}"; do
                for i in "${!services[@]}"; do
                    if [[ ${services[i]} == $target ]]; then
                        warn "Cannot run \"${services[i]}\" service: Nvidia runtime was not found."
                        unset 'services[i]'
                    fi
                done
            done
        fi
        if (( ${#services[@]} != 0 )); then
            for service in ${services[@]}; do
                $LAUNCHER $LAUNCHER_ARGS up --remove-orphans "$service"
            done
        fi
    else
        info "No service data needs update"
    fi
}

_get_hardware_info() {
    if [[ $AWS_REGION_NAME == cn-* ]]; then
        export AWS_DEFAULT_REGION=cn-north-1
    else
        export AWS_DEFAULT_REGION=ap-east-1
    fi
    info "Updating hardware info"
    aws s3 sync --delete --only-show-errors \
        --exclude "*" \
        --include "configs.tar.gz" \
        s3://dl.hrsdk.hansonrobotics.com/${CONFIG_NAME}/ \
        $HR_BOOTSTRAP_DIR || true
    if [[ ! -f $HR_BOOTSTRAP_DIR/configs.tar.gz ]]; then
        error "Couldn't download hardware configs"
        exit 1
    fi
    if [[ -d $HR_BOOTSTRAP_DIR/configs ]]; then
        rm -r $HR_BOOTSTRAP_DIR/configs
    fi
    mkdir -p $HR_BOOTSTRAP_DIR/configs
    tar zxf $HR_BOOTSTRAP_DIR/configs.tar.gz --strip-components 1 -C $HR_BOOTSTRAP_DIR/configs
}

_docker_backup() {
    local image=$1
    docker tag ${image} ${image}.bak
}

_docker_pull() {
    local image=$1
    docker tag ${image} ${image}_ >/dev/null 2>&1 || true
    docker rmi ${image} >/dev/null 2>&1 || true
    docker pull ${image}
    docker rmi ${image}_ >/dev/null 2>&1 || true
}

_pull_images() {
    get_config_schema
    info "Schema $schema_number"

    local image images
    case "$schema_number" in
        0|0.5|0.6)
            # old version file format
            if [[ -f $VERSION_FILE ]]; then
                if [[ $PULL_BACKUP == 1 ]]; then
                    while read -r line; do
                        [[ $line =~ ^#.* ]] && continue
                        line=$(echo $line | xargs)  # strip whitespaces
                        [[ -z $line ]] && continue
                        repo=$(echo $line | cut -d= -f2)
                        image=$ECR_URL/$repo  # replace the ECR_URL
                        _docker_backup ${image}
                    done < ${VERSION_FILE}
                    info "Ran backup successfully"
                fi
                while read -r line; do
                    [[ $line =~ ^#.* ]] && continue
                    line=$(echo $line | xargs)  # strip whitespaces
                    [[ -z $line ]] && continue
                    repo=$(echo $line | cut -d= -f2)
                    image=$ECR_URL/$repo  # replace the ECR_URL
                    _docker_pull ${image}
                done < ${VERSION_FILE}
            fi
            ;;
        1.0)
            if [[ -z $environment ]]; then
                fail "Specify --images <environment>"
            fi
            if [[ -f ${VERSION_FILE}.json && ! -z $environment ]]; then
                local check_environment=$(cat ${VERSION_FILE}.json | jq -r ".environments.\"${environment}\"")
                if [[ $check_environment == null ]]; then
                    fail "No such environment \"$environment\""
                fi
                local services=$(cat ${VERSION_FILE}.json | jq -r ".environments.\"${environment}\"[]")

                for service in ${services[@]}; do
                    image=$(get_image_by_service $service)
                    if [[ $_image == null:null ]]; then
                        error "Service $service was not found"
                        continue
                    fi
                    image=$ECR_URL/$image
                    images+=( ${image} )
                done

                # remove duplicted images
                images=( $(printf '%s\n' "${images[@]}" | sort -u) )

                if [[ $PULL_BACKUP == 1 ]]; then
                    for image in ${images[@]}; do
                        _docker_backup ${image}
                    done
                    info "Ran backup successfully"
                fi

                info "Updating images"
                for image in ${images[@]}; do
                    info ${image}
                    _docker_pull ${image}
                done
            fi
            ;;
        1.1)
            if [[ -z $environment ]]; then
                fail "Specify --images <environment>"
            fi
            images=( $(get_images_by_service_group $environment) )
            if [[ -z $images ]]; then
                fail "Couldn't find images to pull in the environment \"${environment}\""
            fi
            if [[ $PULL_BACKUP == 1 ]]; then
                for image in ${images[@]}; do
                    _docker_backup ${image}
                done
                info "Ran backup successfully"
            fi

            info "Updating images"
            for image in ${images[@]}; do
                info ${image}
                _docker_pull ${image}
            done
            ;;
        *)
            fail "Unknown config schema"
            ;;
    esac
}

if [[ $(readlink -f ${BASH_SOURCE[0]}) == $(readlink -f $0) ]]; then
    set -e

    source $(dirname $(readlink -f ${BASH_SOURCE[0]}))/config.sh
    parse_args "$@"

    update_env # run once before

    info "Getting hrsdk info"
    datetime=$(date +"%Y%m%dT%H:%M:%S")
    info_dir=$USER_CONFIG_DIR/hrsdk.info/$(hostname)
    mkdir -p $info_dir
    /usr/bin/hrsdk info >$info_dir/${datetime}.info 2>/dev/null || true

    if [[ $NO_BOOTSTRAP_FILES != 1 ]]; then
        _get_bootstrap_files
        update_env # and run it after fetching the new config files
    fi

    if [[ $PULL_SDK_IMAGES == 1 ]]; then
        if [[ $ECR_URL == *.amazonaws.com || $ECR_URL == *.amazonaws.com.cn ]]; then
            login_ecr || fail "Login ECR failed"
        fi
        _pull_images
    fi
    if [[ $PULL_SDK_CONFIGS == 1 || $PULL_SDK_MODELS == 1 ]]; then
        if [[ $PULL_SDK_CONFIGS == 1 ]]; then
            _get_hardware_info
        fi
        _get_storage # storage image needs to update first
        if [[ $PULL_SDK_MODELS == 1 ]]; then
            _get_service_data
        fi
    fi
    info "Finished"
fi
