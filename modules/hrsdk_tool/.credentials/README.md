# Credentials Setup

This directory should contain your credential files required to run the Hanson Robotics Robot SDK Tool. 
For security reasons, actual credentials are not included in the repository.

## Required Credential Files

1. **Google Cloud Service Accounts:**
   - `virtual-sophia-12cc84bc05e8.json` - Service account for Virtual Sophia
   - `speechkey.json` - Service account for speech recognition/synthesis

2. **AWS Credentials:**
   - `aws_hrsdk` - AWS credentials for HRSDK
   - `aws_xprize` - AWS credentials for XPrize

3. **Chatbot API Keys:**
   - `chatbot` - Contains API keys for XiaoI and Baidu UNIT

4. **Airtable Credentials:**
   - `airtable` - Contains Airtable API key and base IDs for content management

5. **Kubernetes Configuration:**
   - `kube_config` - Configuration for accessing the Kubernetes cluster

## Setup Instructions

1. Copy the template files from the `templates` directory
2. Rename and place them in the `.credentials` directory
3. Fill in your actual credentials
