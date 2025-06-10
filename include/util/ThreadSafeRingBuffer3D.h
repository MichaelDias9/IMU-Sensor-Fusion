#pragma once
#include <array>
#include <cstddef>
#include <mutex>
#include <algorithm>
#include <stdexcept>

template <std::size_t Capacity>
class ThreadSafeRingBuffer3D {
public:
    ThreadSafeRingBuffer3D() : head(0), count(0) {}

    void append(const float* xData, const float* yData, const float* zData, std::size_t len) {
        if (len > Capacity) {
            throw std::length_error("Buffer cannot fit data");
        }
        std::lock_guard<std::mutex> lock(mtx);

        if (head + len <= 2 * Capacity) {   // No overflow
            std::copy(xData, xData + len, xBuffer.begin() + head);
            std::copy(yData, yData + len, yBuffer.begin() + head);
            std::copy(zData, zData + len, zBuffer.begin() + head);

            head += len;
            count = std::min(count + len, Capacity);
        } else {                            // Overflow
            std::size_t overflowing_count = head + len - 2 * Capacity;
            std::size_t elements_to_keep = Capacity - len;
            std::copy(xBuffer.begin() + (head - elements_to_keep), 
                      xBuffer.begin() + head, 
                      xBuffer.begin());
            std::copy(xData, xData + len, xBuffer.begin() + elements_to_keep);

            std::copy(yBuffer.begin() + (head - elements_to_keep), 
                      yBuffer.begin() + head, 
                      yBuffer.begin());
            std::copy(yData, yData + len, yBuffer.begin() + elements_to_keep);

            std::copy(zBuffer.begin() + (head - elements_to_keep), 
                      zBuffer.begin() + head, 
                      zBuffer.begin());
            std::copy(zData, zData + len, zBuffer.begin() + elements_to_keep);
            head = Capacity;
            count = Capacity;
        }
    }

    std::size_t size() const {
        std::lock_guard<std::mutex> lock(mtx);
        return count;
    }

    void getRecentPointers(std::size_t N, const float** x, const float** y, const float** z, std::size_t headIndex = SIZE_MAX) const {
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
                *x = nullptr;
                *y = nullptr;
                *z = nullptr;
                return;
            }
        } else {
            // Original validation for current head
            if (count < N) {
                *x = nullptr;
                *y = nullptr;
                *z = nullptr;
                return; 
            }
        }

        *x = xBuffer.data() + (useHead - N);
        *y = yBuffer.data() + (useHead - N);
        *z = zBuffer.data() + (useHead - N);
    }

    float atX(std::size_t index) const {
        std::lock_guard<std::mutex> lock(mtx);
        return xBuffer[index];
    }

    float atY(std::size_t index) const {
        std::lock_guard<std::mutex> lock(mtx);
        return yBuffer[index];
    }

    float atZ(std::size_t index) const {
        std::lock_guard<std::mutex> lock(mtx);
        return zBuffer[index];
    }

    std::size_t getHead() const {
        std::lock_guard<std::mutex> lock(mtx);
        return head;
    }

private:
    mutable std::mutex mtx;
    std::array<float, 2 * Capacity> xBuffer{};
    std::array<float, 2 * Capacity> yBuffer{};
    std::array<float, 2 * Capacity> zBuffer{};
    std::size_t head;
    std::size_t count;
};