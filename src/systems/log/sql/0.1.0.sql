/*
 * Copyright (C) 2019 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

/* Note: Use PRAGMA foreign_keys = ON; prior to writing to a database using this
   schema */

/* Describes the schema version used in this database */
CREATE TABLE migrations (
  /* Uniquely identifies a row in this table.
     Sqlite3 will make it an alias of rowid. */
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  /* Previous schema version.
     NULL on the row inserted when the database is created. */
  from_version TEXT DEFAULT NULL,
  /* Version of the schema the database was migrated to. */
  to_version TEXT NOT NULL,
  /* Time when the migration happened (auto populates). */
  time_utc INTEGER NOT NULL DEFAULT CURRENT_TIMESTAMP
);

/* Set the initial version to 0.1.0 */
INSERT INTO migrations (to_version) VALUES ('0.1.0');

/* Contains every message received on every topic recorded */
CREATE TABLE messages (
  /* Uniquely identifies a row in this table.
     Sqlite3 will make it an alias of rowid. */
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  /* Timestamp the message was received (utc nanoseconds) */
  time_recv INTEGER NOT NULL,
  /* TODO: Replace this with a blob type */
  /* A value used for testing */
  value INTEGER NOT NULL
);

/* Lots of queries are done by time received, so add an index to speed it up */
CREATE INDEX idx_time_recv ON messages (time_recv);
