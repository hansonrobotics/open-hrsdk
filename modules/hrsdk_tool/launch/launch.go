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

package main

import (
	"bufio"
	"fmt"
	"io"
	"os"
	"os/exec"
	"os/signal"
	"strings"
	"syscall"
	"time"
)

// ISSUER the issuer in the claim of the payload
const ISSUER = "hansonrobotics"

// SUBJECT the subject in the claim of the payload
const SUBJECT = "hrsdk"

// CustomPayload the custom payload the allows custom fields
type CustomPayload struct {
	jwt.Payload
	Scope string `json:"scope,omitempty"`
	// ConfigName string `json:"configname,omitempty"`
}

var hs = jwt.NewHS256([]byte("UOzViWOqTY"))

func atexit() {
	c := make(chan os.Signal)
	signal.Notify(c, os.Interrupt, syscall.SIGTERM)
	go func() {
		<-c
		fmt.Println("Ctrl+C pressed in Terminal")
	}()
}

func print(stdout io.ReadCloser) {
	r := bufio.NewReader(stdout)
	for {
		line, _, err := r.ReadLine()
		if err == io.EOF {
			return
		}
		fmt.Printf("%s\n", line)
	}
}

func check(e error) {
	if e != nil {
		panic(e)
	}
}

func main() {
	// launch SDK
	atexit()
	cmd := exec.Command("/opt/hansonrobotics/hrsdk/bin/_launch", os.Args[1:len(os.Args)]...)
	cmd.Env = os.Environ()
	stdout, _ := cmd.StdoutPipe()
	stderr, _ := cmd.StderrPipe()
	cmd.Start()
	go print(stdout)
	go print(stderr)
	cmd.Wait()
}
