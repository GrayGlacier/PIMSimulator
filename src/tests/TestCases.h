/***************************************************************************************************
 * Copyright (C) 2021 Samsung Electronics Co. LTD
 *
 * This software is a property of Samsung Electronics.
 * No part of this software, either material or conceptual may be copied or distributed,
 * transmitted, transcribed, stored in a retrieval system, or translated into any human
 * or computer language in any form by any means,electronic, mechanical, manual or otherwise,
 * or disclosed to third parties without the express written permission of Samsung Electronics.
 * (Use of the Software is restricted to non-commercial, personal or academic, research purpose
 * only)
 **************************************************************************************************/

#ifndef __TEST_CASES_H__
#define __TEST_CASES_H__

#include <stdio.h>
#include <iostream>
#include <memory>
#include <string>

#include "Burst.h"
#include "FP16.h"
#include "gtest/gtest.h"
#include "tests/PIMKernel.h"
#include "Callback.h"
#include <stdint.h> 

#include <fstream>
#include <iostream>

using namespace DRAMSim;

class basicFixture : public testing::Test
{
  public:
    basicFixture() {}
    ~basicFixture() {}
    virtual void SetUp() {}
    virtual void TearDown() {}
};

class HeterogenousMemoryFixture: public testing::Test
{
    public:
        HeterogenousMemoryFixture() {}
        ~HeterogenousMemoryFixture() {}
    virtual void SetUp() 
    {
        cur_cycle = 0;

        hbm_callback = MyCallBack();
        ddr4_callback = MyCallBack();


        hbm_mem = make_shared<MultiChannelMemorySystem>(
            "ini/HBM2_samsung_2M_16B_x64.ini", "system_hbm.ini", ".", "example_app",
            256 * 16);


        ddr4_mem = make_shared<MultiChannelMemorySystem>(
            "ini/HBM2_samsung_2M_16B_x64.ini", "system_hbm_64ch.ini", ".", "example_app",
            256 * 64 * 2);

        hbm_mem->RegisterCallbacks(&hbm_callback, NULL, NULL);
        ddr4_mem->RegisterCallbacks(&ddr4_callback, NULL, NULL);

        mem_size = (uint64_t)getConfigParam(UINT, "NUM_CHANS") * getConfigParam(UINT, "NUM_RANKS") *
                   getConfigParam(UINT, "NUM_BANK_GROUPS") * getConfigParam(UINT, "NUM_BANKS") *
                   getConfigParam(UINT, "NUM_ROWS") * getConfigParam(UINT, "NUM_COLS");

        basic_stride =
            (getConfigParam(UINT, "JEDEC_DATA_BUS_BITS") * getConfigParam(UINT, "BL") / 8);

        getMemTraffic("/home/youngsuk95/PIMSimulator/src/tests/trace.txt");

    }

    class MyCallBack : public TransactionCompleteCB
    {
        public:
            MyCallBack() 
            {
                complete_addr = 0;
                complete_cycle = 0;
            }

            void operator()(unsigned id, uint64_t addr, uint64_t cycle)
            {
                complete_addr = addr;
                complete_cycle = cycle;
            }

            uint64_t complete_addr;
            uint64_t complete_cycle;

    };

    void setDataSize(unsigned size)
    {
        data_size_in_byte = size;
    }

    void addTransactionPerPooling(int pooling_idx, int is_write, BurstType bst)
    {
        for(int j=0;j<HBM_transaction[pooling_idx].size();j++)
        {
            hbm_mem->addTransaction(is_write, HBM_transaction[pooling_idx][j], &bst);
        }
        for(int k=0;k<DIMM_transaction[pooling_idx].size();k++)
        {
            ddr4_mem->addTransaction(is_write, DIMM_transaction[pooling_idx][k], &bst);
        }
    }

    void executeSpace()
    {
        bool is_write = false;
        BurstType nullBst;
        int add_cycle = 3;
        int nmp_cycle_left = add_cycle;
        bool is_calculating = false;

        int target_addr = 0;
        int nmp_tail_cycles = 0;
        int pooling_count = 0;

        int buffer_queue=0;
        uint64_t last_addr = 0;

        int total_embedding = HBM_transaction.size();

        for(int i=0; i<total_embedding; i++)
        {
            pooling_count = HBM_transaction[i].size() + DIMM_transaction[i].size();
            addTransactionPerPooling(i, is_write, nullBst);
            uint64_t hbm_target_addr = HBM_transaction[i][0];
            uint64_t ddr4_target_addr = DIMM_transaction[i][0];
            int hbm_addr_idx = 0;
            int ddr4_addr_idx = 0;

            while (hbm_mem->hasPendingTransactions() || ddr4_mem->hasPendingTransactions())
            {   
                hbm_mem->update();
                ddr4_mem->update();

                if(is_calculating)
                {
                    nmp_cycle_left--;
                    if(nmp_cycle_left == 0)
                        is_calculating = false;
                }

                if (hbm_callback.complete_addr == hbm_target_addr)
                {
                    pooling_count -= 1;
                    hbm_addr_idx++;
                    hbm_target_addr = HBM_transaction[i][hbm_addr_idx];
                    if(is_calculating)
                        buffer_queue++;
                    else
                        is_calculating = true;
                }
                if (ddr4_callback.complete_addr == ddr4_target_addr)
                {
                    pooling_count -= 1;
                    ddr4_addr_idx++;
                    ddr4_target_addr = DIMM_transaction[i][ddr4_addr_idx];
                    if(is_calculating)
                        buffer_queue++;
                    else
                        is_calculating = true;

                }
                if (pooling_count == 0)
                    nmp_tail_cycles += 3;

                if (buffer_queue > 0)
                {
                    if(!is_calculating)
                        nmp_cycle_left = add_cycle;
                }
                // cout << hbm_callback.complete_addr << endl;
                // cout << hbm_callback.complete_cycle << endl;
            }

        }


    }

    void getMemTraffic(string filename)
    {
        string line, str_tmp;
        std::ifstream file(filename);
        std::stringstream ss;
        int count = 0;

        std::vector<uint64_t> inner1;
        std::vector<uint64_t> inner2;
        HBM_transaction.push_back(inner1);
        DIMM_transaction.push_back(inner2);
        HBM_transaction[count].clear();
        DIMM_transaction[count].clear();
        cout << filename << endl;

        if(file.is_open())
        {
            while (std::getline(file, line, '\n'))
            {
                if(line.empty())
                {
                    count++;
                    std::vector<uint64_t> inner1;
                    std::vector<uint64_t> inner2;
                    HBM_transaction.push_back(inner1);
                    DIMM_transaction.push_back(inner2);
                    HBM_transaction[count].clear();
                    DIMM_transaction[count].clear();
                    continue;
                }

                ss.str(line);
                string mode = "DIMM";
                uint64_t addr = 0;
                int str_count = 0;

                while (ss >> str_tmp)
                {
                    if(str_count == 0)
                    {
                        if(str_tmp == "\n")
                            count++;
                        else
                            mode = str_tmp;
                    }
                    else
                        addr = std::stoull(str_tmp);

                    str_count++;
                }

                if(mode == "HBM")
                    HBM_transaction[count].push_back(addr);
                else
                    DIMM_transaction[count].push_back(addr);

                ss.clear();
            }

            // for(int i=0;i<HBM_transaction.size();i++)
            // {
            //     vector<uint64_t> trans = HBM_transaction[i];
            //     for(int j=0;j<trans.size();j++)
            //     {
            //         cout << trans[j] << endl;
            //     }
            // }

        }
           
    }

    void generateDummyMemTraffic(bool is_write)
    {
        int num_trans = 0;
        BurstType nullBst;

        cout << "test start" << endl;
        cout << mem_size << endl;

        for (uint64_t i = 0; i < mem_size; ++i)
        {
            if (num_trans >= (data_size_in_byte / basic_stride))
            {
                break;
            }

            // uint64_t addr = i * basic_stride;
            uint64_t addr = 0;
            hbm_mem->addTransaction(is_write, addr, &nullBst);
            hbm_mem->addTransaction(is_write, addr, &nullBst);
            // ddr4_mem->addTransaction(is_write, addr, &nullBst);
            num_trans++;
        }

        while (hbm_mem->hasPendingTransactions())
        {
            cur_cycle++;
            hbm_mem->update();
            // cout << hbm_callback.complete_addr << endl;
            // cout << hbm_callback.complete_cycle << endl;

        }

    }

    private:
        shared_ptr<MultiChannelMemorySystem> hbm_mem;
        shared_ptr<MultiChannelMemorySystem> ddr4_mem;
        bool write_;
        uint64_t cur_cycle = 0;
        uint64_t mem_size;
        uint64_t data_size_in_byte;
        uint64_t basic_stride;
        MyCallBack hbm_callback;
        MyCallBack ddr4_callback;

        vector <vector<uint64_t>> HBM_transaction;
        vector <vector<uint64_t>> DIMM_transaction;


};



class MemBandwidthFixture : public testing::Test
{
  public:
    MemBandwidthFixture() {}
    ~MemBandwidthFixture() {}
    virtual void SetUp()
    {
        cur_cycle = 0;
        mem = make_shared<MultiChannelMemorySystem>("ini/HBM2_samsung_2M_16B_x64.ini",
                                                    "system_hbm.ini", ".", "example_app", 256 * 16);
        mem_size = (uint64_t)getConfigParam(UINT, "NUM_CHANS") * getConfigParam(UINT, "NUM_RANKS") *
                   getConfigParam(UINT, "NUM_BANK_GROUPS") * getConfigParam(UINT, "NUM_BANKS") *
                   getConfigParam(UINT, "NUM_ROWS") * getConfigParam(UINT, "NUM_COLS");
        basic_stride =
            (getConfigParam(UINT, "JEDEC_DATA_BUS_BITS") * getConfigParam(UINT, "BL") / 8);
    }
    virtual void TearDown()
    {
        printResult(cur_cycle);
    }

    uint64_t measureCycle(bool is_write)
    {
        printTestMessage();
        write_ = is_write;
        generateMemTraffic(is_write);

        while (mem->hasPendingTransactions())
        {
            cur_cycle++;
            mem->update();
        }

        return cur_cycle;
    }

    void printTestMessage()
    {
        cout << ">> Bandwidth Test" << endl;
        cout << "  Num. channels: " << getConfigParam(UINT, "NUM_CHANS") << endl;
        cout << "  Num. ranks: " << getConfigParam(UINT, "NUM_RANKS") << endl;
        cout << "  Num. banks: " << getConfigParam(UINT, "NUM_BANKS") << endl;
        cout << "  Data size (byte): " << data_size_in_byte << endl;
    }

    void printResult(uint64_t cycle)
    {
        uint64_t totalReads = 0;
        uint64_t totalWrites = 0;

        for (size_t i = 0; i < getConfigParam(UINT, "NUM_CHANS"); i++)
        {
            MemoryController* mem_ctrl = mem->channels[i]->memoryController;
            totalReads += mem_ctrl->totalReads;
            totalWrites += mem_ctrl->totalWrites;
        }
        uint32_t bw = (totalReads + totalWrites) * getConfigParam(UINT, "JEDEC_DATA_BUS_BITS") *
                      getConfigParam(UINT, "BL") / 8 / (cycle * getConfigParam(FLOAT, "tCK"));
        cout << endl;
        cout << "> Test Result " << endl;
        cout << "> BW (GB/s): " << bw << endl;
    }

    void setDataSize(unsigned size)
    {
        data_size_in_byte = size;
    }

    void generateMemTraffic(bool is_write)
    {
        int num_trans = 0;
        BurstType nullBst;

        for (uint64_t i = 0; i < mem_size; ++i)
        {
            if (num_trans >= (data_size_in_byte / basic_stride))
            {
                break;
            }
            uint64_t addr = i * basic_stride;
            // cout << addr << endl;
            mem->addTransaction(is_write, addr, &nullBst);
            num_trans++;
        }
    }

  private:
    bool write_;
    uint64_t cur_cycle = 0;
    uint64_t mem_size;
    uint64_t data_size_in_byte;
    uint64_t basic_stride;
    shared_ptr<MultiChannelMemorySystem> mem;
};

class DataDim
{
  private:
    unsigned getPrecisionToByte()
    {
        switch (PIMConfiguration::getPIMPrecision())
        {
            case INT8:
                return 1;
            case FP16:
                return 2;
            case FP32:
                return 4;
            default:
                return 0;
        }
    }

    void loadData(KernelType kn_type)
    {
        string input_dim_str = to_string(input_dim_);

        switch (kn_type)
        {
            case KernelType::GEMV:
            case KernelType::GEMVTREE:
            {
                string output_dim_str = to_string(output_dim_);

                string in_out_dim_str = output_dim_str + "x" + input_dim_str;

                if (batch_size_ > 1)
                {
                    string batch_size_str = to_string(batch_size_);
                    string batch_in_out_dim_str = batch_size_str + "_" + in_out_dim_str;
                    input_npbst_.loadFp16("data/gemv/gemv_input_batch_" + batch_in_out_dim_str +
                                          ".npy");
                    weight_npbst_.loadFp16("data/gemv/gemv_weight_batch_" + batch_in_out_dim_str +
                                           ".npy");
                    output_npbst_.loadFp16("data/gemv/gemv_output_batch_" + batch_in_out_dim_str +
                                           ".npy");
                }
                else
                {
                    input_npbst_.loadFp16("data/gemv/gemv_input_" + in_out_dim_str + ".npy");
                    weight_npbst_.loadFp16("data/gemv/gemv_weight_" + in_out_dim_str + ".npy");
                    output_npbst_.loadFp16("data/gemv/gemv_output_" + in_out_dim_str + ".npy");
                }

                // output_dim_ = weight_npbst_.bShape[0];
                output_dim_ = bShape1ToDim(output_npbst_.bShape[1]);
                input_dim_ = bShape1ToDim(input_npbst_.bShape[1]);
                batch_size_ = input_npbst_.bShape[0];
                return;
            }
            case KernelType::ADD:
            {
                input_npbst_.loadFp16("data/add/resadd_input0_" + input_dim_str + ".npy");
                input1_npbst_.loadFp16("data/add/resadd_input1_" + input_dim_str + ".npy");
                output_npbst_.loadFp16("data/add/resadd_output_" + input_dim_str + ".npy");

                output_dim_ = bShape1ToDim(output_npbst_.getTotalDim());
                input_dim_ = bShape1ToDim(input_npbst_.getTotalDim());
                input1_dim_ = bShape1ToDim(input1_npbst_.getTotalDim());

                return;
            }
            case KernelType::MUL:
            {
                input_npbst_.loadFp16("data/mul/eltmul_input0_" + input_dim_str + ".npy");
                input1_npbst_.loadFp16("data/mul/eltmul_input1_" + input_dim_str + ".npy");
                output_npbst_.loadFp16("data/mul/eltmul_output_" + input_dim_str + ".npy");

                output_dim_ = bShape1ToDim(output_npbst_.getTotalDim());
                input_dim_ = bShape1ToDim(input_npbst_.getTotalDim());
                input1_dim_ = bShape1ToDim(input1_npbst_.getTotalDim());

                return;
            }
            case KernelType::RELU:
            {
                input_npbst_.loadFp16("data/relu/relu_input_" + input_dim_str + ".npy");
                output_npbst_.loadFp16("data/relu/relu_output_" + input_dim_str + ".npy");

                output_dim_ = bShape1ToDim(output_npbst_.getTotalDim());
                input_dim_ = bShape1ToDim(input_npbst_.getTotalDim());

                return;
            }
            default:
            {
                ERROR("== Error - Unknown KernelType trying to load data");
                exit(-1);
                return;
            }
        }
    }

    void loadDummyData(KernelType kn_type)
    {
        switch (kn_type)
        {
            case KernelType::GEMV:
            case KernelType::GEMVTREE:
            {
                weight_npbst_.shape.push_back(output_dim_);
                weight_npbst_.shape.push_back(input_dim_);
                weight_npbst_.loadTobShape(16);

                input_npbst_.shape.push_back(batch_size_);
                input_npbst_.shape.push_back(input_dim_);
                input_npbst_.loadTobShape(16);

                for (int i = 0; i < input_npbst_.bShape[1]; i++)
                {
                    BurstType null_bst;
                    null_bst.set((float)0);
                    input_npbst_.bData.push_back(null_bst);
                }

                return;
            }
            case KernelType::ADD:
            case KernelType::MUL:
            case KernelType::RELU:
            {
                input_npbst_.shape.push_back(batch_size_);
                input_npbst_.shape.push_back(input_dim_);
                input_npbst_.loadTobShape(16);

                output_npbst_.shape.push_back(batch_size_);
                output_npbst_.shape.push_back(output_dim_);
                output_npbst_.loadTobShape(16);

                return;
            }
            default:
            {
                return;
            }
        }
    }

  public:
    /* data */
    NumpyBurstType input_npbst_;
    NumpyBurstType input1_npbst_;
    NumpyBurstType weight_npbst_;
    NumpyBurstType output_npbst_;

    /* dump */
    NumpyBurstType preloaded_npbst_;
    NumpyBurstType result_npbst_;
    NumpyBurstType reduced_result_npbst_;

    size_t burst_cnt_;
    BurstType* preloaded_bst_;
    BurstType* result_;
    BurstType* reduced_result_;

    /* dim */
    unsigned long output_dim_;
    int input_dim_;
    int input1_dim_;
    int batch_size_;
    bool used_data_;

    DataDim(KernelType kn_type, uint32_t batch_size, uint32_t output_dim, uint32_t input_dim,
            bool used_data)
    {
        batch_size_ = batch_size;
        output_dim_ = output_dim;
        input_dim_ = input_dim;
        used_data_ = used_data;

        switch (kn_type)
        {
            case KernelType::MUL:
            case KernelType::ADD:
            {
                input1_dim_ = input_dim;
                break;
            }
            default:
            {
                break;
            }
        }

        // load data from files
        if (used_data_)
            loadData(kn_type);
        else
            loadDummyData(kn_type);
    }

    uint32_t getDataSize(uint32_t dim1, uint32_t dim2 = 1, uint32_t dim3 = 1)
    {
        return dim1 * dim2 * dim3 * getPrecisionToByte();
    }

    void printDim(KernelType kn_type)
    {
        switch (kn_type)
        {
            case KernelType::GEMV:
            case KernelType::GEMVTREE:
            {
                cout << "  Weight data dimension : " << output_dim_ << "x" << input_dim_ << endl;
                if (batch_size_ > 1)
                {
                    cout << "  Input data dimension : " << input_dim_ << "x" << batch_size_ << endl;
                    cout << "  Output data dimension : " << output_dim_ << "x" << batch_size_
                         << endl;
                }
                else
                {
                    cout << "  Input data dimension : " << input_dim_ << endl;
                    cout << "  Output data dimension : " << output_dim_ << endl;
                }
                break;
            }
            case KernelType::MUL:
            case KernelType::ADD:
            case KernelType::RELU:
            {
                cout << "  Input/output data dimension : " << output_dim_ << endl;
                break;
            }
            default:
            {
                ERROR("== Error - Unknown KernelType trying to load data");
                exit(-1);
                break;
            }
        }
    }
    uint32_t getNumElementsPerBlocks()
    {
        return ((getConfigParam(UINT, "JEDEC_DATA_BUS_BITS") * getConfigParam(UINT, "BL") / 8) /
                getPrecisionToByte());
    }

    uint32_t dimTobShape(int in_dim)
    {
        return ceil(in_dim / getNumElementsPerBlocks());
    }

    uint32_t bShape1ToDim(int bSahpe1)
    {
        return bSahpe1 * getNumElementsPerBlocks();
    }
};

#endif
