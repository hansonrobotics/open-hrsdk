This repository contains the Hanson Robotics Robot SDK Tool.

# SDK Setup
To install the SDK, please refer to the following documentation:
https://hansonrobotics.notion.site/SDK-Setup-e5a021d2059e47309ee6add86d0c8046

# Credentials Setup
This SDK requires several API keys and credentials to function properly:

1. Google Cloud service accounts for speech and Virtual Sophia
2. AWS credentials for storage and processing
3. Chatbot API keys for conversation functionality

Please see the [credentials setup instructions](.credentials/README.md) for details on how to set up your credentials.

**IMPORTANT:** Never commit your actual credentials to the repository. The `.credentials` directory (except for templates and README) is gitignored for your protection.
