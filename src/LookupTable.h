#ifndef LOOKUPTABLE_H_
#define LOOKUPTABLE_H_

#include <iostream>
#include <vector>
#include <cstdint>

namespace DRAMSim
{
    class LookupTable {
    public:
        LookupTable(size_t num_buffers) : buffers(num_buffers), buffer_size(num_buffers), counter(0) {}

        bool writeToLookupTable(uint64_t address) {
            if (counter < buffer_size)
            {
                buffers[counter] = address;
                counter++;
                return true;
            }
            else
                return false;
        }

        bool Hit(uint64_t address){
            bool hit = false;
            for (int i=0; i<buffer_size; i++)
            {
                if (buffers[i] == address)
                {
                    hit = true;
                    break;
                }

            }
            return hit;
        }

        void printLookupTable() const {
            for (size_t i = 0; i < buffers.size(); ++i) {
                std::cout << "Buffer " << i << ": " << buffers[i] << std::endl;
            }
        }

    private:
        std::vector<uint64_t> buffers;
        int buffer_size;
        int counter;
    };

}





















#endif
