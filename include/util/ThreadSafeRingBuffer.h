#pragma once
#include <array>
#include <cstddef>
#include <mutex>
#include <algorithm>
#include <stdexcept>

template <std::size_t Capacity>
class ThreadSafeRingBuffer {
public:
    ThreadSafeRingBuffer() : head(0), count(0) {}

    void append(const float* data, std::size_t len) {
        if (len > Capacity) {
            throw std::length_error("Buffer cannot fit data");
        }
        std::lock_guard<std::mutex> lock(mtx);

        if (head + len <= 2 * Capacity) {   // No overflow
            std::copy(data, data + len, buffer.begin() + head);
            head += len;
            count = std::min(count + len, Capacity);
        } else {                            // Overflow
            std::size_t overflowing_count = head + len - 2 * Capacity;
            std::size_t elements_to_keep = Capacity - len;
            std::copy(buffer.begin() + (head - elements_to_keep), 
                      buffer.begin() + head, 
                      buffer.begin());
            std::copy(data, data + len, buffer.begin() + elements_to_keep);
            head = Capacity;
            count = Capacity;
        }
    }

    void append(const float data) {
        std::lock_guard<std::mutex> lock(mtx);

        if (head + 1 <= 2 * Capacity) {   // No overflow
            buffer[head] = data;
            head += 1;
            count = std::min(count + 1, Capacity);
        } else {                           // Overflow
            std::size_t elements_to_keep = Capacity - 1;
            std::copy(buffer.begin() + (head - elements_to_keep), 
                      buffer.begin() + head, 
                      buffer.begin());
            buffer[elements_to_keep] = data;
            head = Capacity;
            count = Capacity;
        }
    }

    float at(std::size_t index) const {
        std::lock_guard<std::mutex> lock(mtx);
        return buffer[index];
    }

    std::size_t size() const {
        std::lock_guard<std::mutex> lock(mtx);
        return count;
    }

    void getRecentPointer(std::size_t N, const float** data, std::size_t headIndex = SIZE_MAX) const {
        if (N > Capacity) {
            throw std::out_of_range("Requested more than buffer capacity");
        }

        std::lock_guard<std::mutex> lock(mtx);
        
        // Use current head if no specific head index was provided
        std::size_t useHead = (headIndex == SIZE_MAX) ? head : headIndex;
        
        // Validation for provided head index
        if (headIndex != SIZE_MAX) {
            if (headIndex > 2 * Capacity) {
                throw std::out_of_range("Head index out of range");
            }
            if (N > headIndex) {
                throw std::out_of_range("Requested more elements than available from head index");
            }
            // Check if the provided head index is still valid (hasn't been overwritten)
            if (headIndex > head) {
                *data = nullptr;
                return;
            }
        } else {
            // Original validation for current head
            if (count < N) {
                *data = nullptr;
                return; 
            }
        }

        *data = buffer.data() + (useHead - N);
    }

    std::size_t getHead() const {
        std::lock_guard<std::mutex> lock(mtx);
        return head;
    }

private:
    mutable std::mutex mtx;
    std::array<float, 2 * Capacity> buffer{};
    std::size_t head;
    std::size_t count;
};