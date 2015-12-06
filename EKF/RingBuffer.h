/****************************************************************************
 *
 *   Copyright (C) 2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file RingBuffer.h
 * @author Roman Bapst <bapstroman@gmail.com>
 * Template RingBuffer.
 */

#include <cstdio>
#include <cstring>

template <typename data_type>
class RingBuffer
{
public:
	RingBuffer()
	{
		_buffer = NULL;
		_head = _tail = _size = 0;
	}
	~RingBuffer() {delete _buffer;}

	bool allocate(int size)
	{
		if (size <= 0) {
			return false;
		}

		if (_buffer != NULL) {
			delete _buffer;
		}

		_buffer = new data_type[size];

		if (_buffer == NULL) {
			return false;
		}

		_size = size;
		return true;
	}

	inline void push(data_type sample, bool debug = false)
	{
		if (debug) {
			printf("elapsed %i\n", sample.time_us - _time_last);
			_time_last = sample.time_us;
		}

		int head_new = (_head + 1) % _size;
		_buffer[head_new] = sample;
		_head = head_new;

		// move tail if we overwrite it
		if (_head == _tail && _size > 1) {
			_tail = (_tail + 1) % _size;
		}
	}

	inline data_type get_oldest()
	{
		return _buffer[_tail];
	}

	inline data_type get_newest()
	{
		return _buffer[_head];
	}

	inline bool pop_first_older_than(uint64_t timestamp, data_type *sample)
	{
		// start looking from newest observation data
		for (unsigned i = 0; i < _size; i++) {
			int index = (_head - i);
			index = index < 0 ? _size + index : index;

			if (timestamp >= _buffer[index].time_us && timestamp - _buffer[index].time_us < 100000) {

				memcpy(sample, &_buffer[index], sizeof(*sample));

				// Now we can set the tail to the item which comes after the one we removed
				// since we don't want to have any older data in the buffer
				if (index == _head) {
					_tail = _head;

				} else {
					_tail = (index + 1) % _size;
				}

				_buffer[index].time_us = 0;

				return true;
			}

			if (index == _tail) {
				// we have reached the tail and haven't got a match
				return false;
			}
		}

		return false;
	}

	data_type &operator[](unsigned index)
	{
		return _buffer[index];
	}

private:
	data_type *_buffer;
	unsigned _head, _tail, _size;

	// debug
	uint64_t _time_last;
};
