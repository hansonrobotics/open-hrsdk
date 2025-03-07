
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

SELECT
    created_at AS Time,
    req.conversation_id AS ConversationID,
    'Input' AS Type,
    req.user_id AS Name,
    text AS Text,
    TRIM('"' FROM JSON_EXTRACT(req.attachment, '$.source')) AS Source
FROM
    chat_requests req
UNION SELECT
    published_at AS time,
    resp.conversation_id,
    'Output',
    UCASE(TRIM('"' FROM JSON_EXTRACT(resp.attachment, '$.robot'))),
    text,
    agent_id AS agent
FROM
    chat_responses resp
WHERE
    resp.published_at IS NOT NULL
ORDER BY time , ConversationID;