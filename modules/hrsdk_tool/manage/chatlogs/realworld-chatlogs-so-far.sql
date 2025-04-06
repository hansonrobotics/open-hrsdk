/*
// Copyright (C) 2017-2025 Hanson Robotics
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

USE chatlogs;
SELECT
    request.request_id,
    request.time,
    request.question,
    request.lang,
    request.robot,
    request.body,
    request.location,
    IF(request.audio <> '',
        CONCAT('https://s3.ap-northeast-2.amazonaws.com/mediastore.hansonrobotics.com/sdk-speech/',
                request.audio),
        '') AS audio,
    response.answer AS agent_answer,
    response.agent_id AS response_agent,
    response.attachment,
    response.trace,
    publish.answer AS publish_answer,
    publish.agent_id AS publish_agent,
    publish.label
FROM
    ChatRequest request
        INNER JOIN
    ChatResponse response USING (request_id)
        INNER JOIN
    PublishedResponse publish USING (request_id)
WHERE response.agent_id = publish.agent_id
ORDER BY request.time ASC
LIMIT 100000;
