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

# Dump the chatlogs by IP
#

dump_by_ip() {
    local client_ip=121.128.171.195
    mysql -uroot -ppassword -h127.0.0.1 -e "use interaction; select uid, req.created_at, req.question, resp.answer, req.lang, client_ip, product, tts, resp.agent_id from chat_request as req inner join chat_response as resp on req.request_id = resp.request_id where resp.published_at is not null and req.client_ip=\"$client_ip\" order by req.created_at limit 10000" >out
    echo "dumped to out"
}

dump_all() {
    mysql -uroot -ppassword -h127.0.0.1 -e "use interaction; select uid, req.created_at, req.question, resp.answer, req.lang, client_ip, product, tts, resp.agent_id from chat_request as req inner join chat_response as resp on req.request_id = resp.request_id where resp.published_at is not null order by req.created_at limit 10000" >out
    echo "dumped to out"
}
dump_all
