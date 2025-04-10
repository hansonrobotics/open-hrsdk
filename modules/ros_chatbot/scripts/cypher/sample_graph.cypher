//
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
//

// create character nodes
CREATE (Sophia:Character:BasePerson {uid: "sophia", name:"Sophia the Robot", born:2016, languages: ["English, Mandarin"]})
// create person nodes
CREATE (Ben:Person:BasePerson {uid: "ben", name:"Ben Goertzel", born:1966, languages: ["English"], gender: "male"})
CREATE (DavidH:Person:BasePerson {uid: "davidh", name:"David Hanson", born:1969, languages: ["English"], gender: "male"})
// create session nodes
CREATE (SessionWithDavidH:Session {uid: "1", time: 1612775757, series: "test series", episode: "test episode", act: "test act", scene: "test scene"})
