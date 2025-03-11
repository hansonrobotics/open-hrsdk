# SDK S3 Configuration Management

## Configure aws

Configure the profile `hrsdk_admin` in order to pull and push the configs to the S3 bucket.

```
aws configure --profile hrsdk_admin

AWS Access Key ID: <access_key_id>
AWS Secret Access Key: <secret_access_key>
Default region name [None]: <region>
Default output format [None]: json
```

## Pull configs

`./download.sh <folder>`

For example `./download.sh xprize`. It will download the configuration files from the folder `xprize` in the S3 bucket to the local location `/tmp/xprize`.

## Push configs

`./sync.sh <src path>`

For example, `./sync.sh /tmp/xprize`. It will synchronize all the files in the local folder `/tmp/xprize` with the folder `xprize` in the S3 bucket.
