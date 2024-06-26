// Copyright 2018 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.
//
// Author: Michael Schaffner <schaffner@iis.ee.ethz.ch>, ETH Zurich
// Date: 26.11.2018
// Description: Simpe test program that writes a block of data to memory, reads it
// back and checks whether the checksum is correct.
//

#include <stdio.h>

// 64 bytes of data
// 64 KB per core
// 4 way 
// 1024 lines
// 1024 / 4 = 256 sets
// ---- | index (8) | offset (6)
// 1 << 14 bytes -> convert to int, right shift by 2 (4 bytes)

#define LOOP_DEPTH 32
#define ITER_OFFSET ((1<<14) >> 2)
#define NUM_WORDS (LOOP_DEPTH * ITER_OFFSET) // should have quite a few writes

int main(int argc, char ** argv) {

  volatile int tmp[NUM_WORDS];
  int accu;


  for (int k = 0; k < LOOP_DEPTH; k++) {
    tmp[k * ITER_OFFSET] = k;
  }

  for (int k = 0; k < LOOP_DEPTH; k++) {
    tmp[k * ITER_OFFSET + k] = 2*k;
  }

  accu = 0;
  for (int k = 0; k < LOOP_DEPTH; k++) {
    if (tmp[k*ITER_OFFSET+k] != k*2) {
		printf("test failed at %d with address %p\n", k, &(tmp[k*ITER_OFFSET+k]));
		return -1;
	}
  }

  return 0;
}
