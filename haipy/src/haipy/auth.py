#
# Copyright (C) 2017-2024 Hanson Robotics
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
#
import json
import logging
import os
import time
from typing import Optional

import boto3
from fastapi import HTTPException, Request, status
from fastapi.security import HTTPAuthorizationCredentials, HTTPBearer
from jose import JWTError, jwk, jwt
from jose.utils import base64url_decode

import haipy.schemas.auth as auth_schemas
from haipy.parameter_server_proxy import GlobalContext

logger = logging.getLogger(__name__)

if "HR_CREDENTIAL_DB" in os.environ:
    with open(os.environ["HR_CREDENTIAL_DB"]) as f:
        credential_db = json.load(f)["auths"]
else:
    logger.error("Credential DB is not found")
    credential_db = {}

token_cache = GlobalContext(ns="global")
cognito_client = boto3.client(
    "cognito-idp", region_name=os.getenv("COGNITO_REGION_NAME")
)


def get_user(access_token):
    return cognito_client.get_user(AccessToken=access_token)


class JWTBearer(HTTPBearer):
    def __init__(self, jwks: auth_schemas.JWKS, auto_error: bool = True):
        super().__init__(auto_error=auto_error)

        self.kid_to_jwk = {jwk["kid"]: jwk for jwk in jwks.keys}

    def verify_jwk_token(
        self, jwt_credentials: auth_schemas.JWTAuthorizationCredentials
    ) -> bool:
        try:
            public_key = self.kid_to_jwk[jwt_credentials.header["kid"]]
        except KeyError:
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN, detail="JWK public key not found"
            )

        key = jwk.construct(public_key)
        decoded_signature = base64url_decode(jwt_credentials.signature.encode())

        return key.verify(jwt_credentials.message.encode(), decoded_signature)

    def get_token(self, token_key):
        # TODO: get jwt token from database
        if token_key not in credential_db:
            if credential_db:
                raise HTTPException(
                    status_code=status.HTTP_403_FORBIDDEN, detail="Invalid key"
                )
            else:
                raise HTTPException(
                    status_code=status.HTTP_403_FORBIDDEN, detail="Credential DB error"
                )
        refresh_token = credential_db[token_key]["refresh_token"]

        try:
            response = cognito_client.initiate_auth(
                ClientId=os.getenv("COGNITO_USER_CLIENT_ID"),
                AuthFlow="REFRESH_TOKEN_AUTH",
                AuthParameters={
                    "REFRESH_TOKEN": refresh_token,
                },
            )
        except Exception as ex:
            logger.error(ex)
            raise HTTPException(status_code=status.HTTP_403_FORBIDDEN, detail=str(ex))
        access_token = response["AuthenticationResult"]["AccessToken"]
        token_cache[token_key] = access_token
        token_cache.expire(token_key, 3600)

    def verify_jwt_token(self, jwt_token) -> auth_schemas.JWTAuthorizationCredentials:
        message, signature = jwt_token.rsplit(".", 1)

        try:
            jwt_credentials = auth_schemas.JWTAuthorizationCredentials(
                jwt_token=jwt_token,
                header=jwt.get_unverified_header(jwt_token),
                claims=jwt.get_unverified_claims(jwt_token),
                signature=signature,
                message=message,
            )
        except JWTError:
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN, detail="Invalid access token"
            )

        if not self.verify_jwk_token(jwt_credentials):
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN, detail="Invalid access token"
            )

        if time.time() > float(jwt_credentials.claims["exp"]):
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN, detail="Access token has expired"
            )

        return jwt_credentials

    async def __call__(
        self, request: Request
    ) -> Optional[auth_schemas.JWTAuthorizationCredentials]:
        credentials: HTTPAuthorizationCredentials = await super().__call__(request)

        if credentials:
            if not credentials.scheme == "Bearer":
                raise HTTPException(
                    status_code=status.HTTP_403_FORBIDDEN,
                    detail="Wrong authentication method",
                )

            token_key = credentials.credentials
            if token_key in token_cache:
                access_token = token_cache[token_key]
            else:
                self.get_token(token_key)
                access_token = token_cache[token_key]

            try:
                jwt_credentials = self.verify_jwt_token(access_token)
            except HTTPException:
                # refresh token
                del token_cache[token_key]
                self.get_token(token_key)
                access_token = token_cache[token_key]
                jwt_credentials = self.verify_jwt_token(access_token)

            return jwt_credentials
