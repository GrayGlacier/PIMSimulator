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

#ifndef __KERNEL_TEST_CASES_H__
#define __KERNEL_TEST_CASES_H__

#include <memory>
#include <vector>

#include "tests/TestCases.h"

using namespace DRAMSim;


#include <chrono>
using namespace std::chrono;

// A predicate-formatter for asserting that two integers are mutually prime.
::testing::AssertionResult fp16EqualHelper(const char* m_expr, const char* n_expr, fp16 m, fp16 n);
::testing::AssertionResult fp16BstEqualHelper(const char* m_expr, const char* n_expr,
                                              DRAMSim::BurstType mb, DRAMSim::BurstType nb);
#define EXPECT_FP16_BST_EQ(val1, val2) EXPECT_PRED_FORMAT2(fp16BstEqualHelper, val1, val2)
#define EXPECT_FP16_EQ(val1, val2) EXPECT_PRED_FORMAT2(fp16EqualHelper, val1, val2)

class TestStats
{
  private:
    unsigned num_test_passed_;
    unsigned num_test_warning_;
    unsigned num_test_failed_;

    vector<unsigned> fail_idx_;
    vector<float> fail_data_sim_;
    vector<float> fail_data_numpy_;

  public:
    TestStats() {}
    unsigned getNumPassed()
    {
        return num_test_passed_;
    }
    unsigned getNumWarning()
    {
        return num_test_warning_;
    }
    unsigned getNumFailed()
    {
        return num_test_failed_;
    }

    void IncNumPassed(unsigned i = 1)
    {
        num_test_passed_ += i;
    }
    void IncNumWarning(unsigned i = 1)
    {
        num_test_warning_ += i;
    }
    void IncNumFailed(unsigned i = 1)
    {
        num_test_failed_ += i;
    }

    void clear()
    {
        num_test_passed_ = 0;
        num_test_warning_ = 0;
        num_test_failed_ = 0;
        fail_idx_.clear();
        fail_data_sim_.clear();
        fail_data_numpy_.clear();
    };

    void insertToFailVector(unsigned idx, float m, float n)
    {
        fail_idx_.push_back(idx);
        fail_data_sim_.push_back(m);
        fail_data_numpy_.push_back(n);
    }

    void printFailVector()
    {
        for (int i = 0; i < fail_idx_.size(); i++)
        {
            cout << " idx : " << fail_idx_[i] << " sim : " << fail_data_sim_[i]
                 << " npy : " << fail_data_numpy_[i] << endl;
        }
    };
};

TestStats testStats;

class MyCallBack : public TransactionCompleteCB
{
    public:
        MyCallBack()
        {
            complete_addr_count = 0;
            complete_cycle = 0;
            erase_queue.clear();
        }

        void operator()(unsigned id, uint64_t addr, uint64_t cycle)
        {
            for(int i=0; i<read_addr_list.size(); i++)
            {
                if(addr == read_addr_list[i])
                {
                    bool duplicate = false;
                    for(int j=0; j<erase_queue.size(); j++)
                    {
                        if(addr == erase_queue[j])
                        {
                            duplicate = true;
                            break;
                        }
                    }
                    if(!duplicate)
                    {
                        complete_channel.push_back(id);
                        complete_addr.push_back(addr);
                        erase_queue.push_back(i);
                        complete_addr_count++;
                        complete_cycle = cycle;
                        // cout << "callback address : " << addr << endl;
                    }
                }                       
            }

            // int erase_count = 0;
            // for(int i=0; i<erase_queue.size(); i++)
            // {
            //     read_addr_list.erase(read_addr_list.begin() + erase_queue[i] - erase_count);
            //     erase_count++;
            // }

        }

        void register_read_addr(uint64_t addr)
        {
            read_addr_list.push_back(addr);
        }

        void complete_addr_check_complete()
        {
            complete_addr.clear();
            complete_channel.clear();
            complete_addr_count = 0;
            erase_queue.clear();
        }

        int complete_addr_count;
        bool scan_complete;
        uint64_t complete_cycle;
        vector<bool> complete_index;
        vector<uint64_t> read_addr_list;
        vector<uint64_t> complete_channel;
        vector<uint64_t> complete_addr;
        vector<int> erase_queue;

};

class MyPIMFixture : public testing::Test
{
    public:
        MyPIMFixture() {}
        ~MyPIMFixture() {}
    virtual void SetUp()
    {
        // Basic memory settings
        hbm_mem = make_shared<MultiChannelMemorySystem>(
            "ini/HBM2_samsung_2M_16B_x64.ini", "system_hbm_16ch.ini", ".", "example_app",
            256 * 16);

        // PIM settings
        int numPIMChan = 16;
        int numPIMRank = 1;
        vec_size_in_byte = 64;
        kernel = make_shared<PIMKernelChannelwise>(hbm_mem, numPIMChan, numPIMRank);
        total_channels = getConfigParam(UINT, "NUM_CHANS");
        total_banks = getConfigParam(UINT, "NUM_BANKS");

        ddr4_mem = make_shared<MultiChannelMemorySystem>(
            "ini/DDR4_8gb_x8_2666.ini", "system_ddr4_1ch.ini", ".", "example_app",
            8*1024);


        // Callback Settings
        hbm_callback = MyCallBack();
        ddr4_callback = MyCallBack();
        hbm_mem->RegisterCallbacks(&hbm_callback, NULL, NULL);
        ddr4_mem->RegisterCallbacks(&ddr4_callback, NULL, NULL);
        ktype = KernelType::EMB;

        // DLRM settings
        getMemTraffic("/home/youngsuk95/PIMSimulator/src/tests/traces/bg_map_trace_col_4.txt");

        // Overall cycle
        cur_cycle = 0;
    }

    uint64_t executePIMPerQemb(int pooling_idx, int ch_idx, int ra_idx, int bg_idx, int bank_idx, int row_idx, int col_idx)
    {
        int input_row = row_idx;
        int result_row = 256;

        // perform q + r addition
        // put RD transaction to read the result
        kernel->executeEltwise(vec_size_in_byte, pimBankType::ALL_BANK, ktype, ch_idx, ra_idx, bank_idx, input_row, result_row);
        uint64_t addr = kernel->readPIMResult(ch_idx, ra_idx, bg_idx, bank_idx, row_idx, col_idx);

        return addr;
    }

    uint64_t executePIMMultipleQembs(int pooling_idx, int ch_idx, int ra_idx, int bg_idx, int bank_idx, vector<int> row_idx, int col_idx, int num_q_embs)
    {
        int result_row = 256;
        kernel->executeEltwiseMultipleQs(vec_size_in_byte, pimBankType::ALL_BANK, ktype, ch_idx, ra_idx, bank_idx, num_q_embs, row_idx, result_row);
        uint64_t addr = kernel->readPIMResult(ch_idx, ra_idx, bg_idx, bank_idx, result_row, col_idx);

        ///////// For Test ////////////////
        // cout << "address : " << addr << endl;

        return addr;
    }

    int determineQembsPerBankPerChannel(int pooling_idx)
    {
        int total_used_banks = 0;
        
        qembs_per_bank.clear();
        qembs_addrs.clear();

        // initialize vectors
        for(int i=0; i<total_channels; i++)
        {
            std::vector<int> dummy_1;
            qembs_per_bank.push_back(dummy_1);
            std::vector<vector<uint64_t>> dummy_2;
            qembs_addrs.push_back(dummy_2);

            for(int j=0; j<total_banks; j++)
            {
                std::vector<uint64_t> dummy_3;
                qembs_addrs[i].push_back(dummy_3);
                qembs_per_bank[i].push_back(0);
            }
        }

        for(int j=0;j<HBM_transaction[pooling_idx].size();j++)
        {
            uint64_t addr = HBM_transaction[pooling_idx][j];
            unsigned chan, rank, bg, bank, row, col;
            kernel->getChanRankBankgroupAddress(addr, chan, rank, bg, bank, row, col);
            qembs_per_bank[chan][bank] = qembs_per_bank[chan][bank] + 1;
            // cout << "channel : " << chan << " total qs : "<< qembs_per_bank[chan][bank] << endl; 
            qembs_addrs[chan][bank].push_back(addr);
        }

        for(int i=0; i<total_channels; i++)
        {
            for(int j=0; j<total_banks; j++)
            {
                if(qembs_per_bank[i][j] > 0)
                {
                    //////////////    DEBUG    //////////////
                    // cout << "channel # : " << i << " bank # : " << j << " vectors : "<< qembs_per_bank[i][j] << endl;
                    total_used_banks++;
                }
            }
        }

        return total_used_banks;
    }

    int addTransactionPerPoolingMultipleQembsWithRespectToChannel(int pooling_idx, bool is_write)
    {
        int total_used_banks = determineQembsPerBankPerChannel(pooling_idx);

        // add transaction with respect to bank
        for(int i=0;i<total_channels;i++)
        {
            for(int j=0;j<total_banks;j++)
            {   
                uint64_t rd_addr;
                vector<int> input_rows;
                unsigned chan, rank, bg, bank, row, col;

                for(int l=0; l<qembs_addrs[i][j].size(); l++)
                {
                    uint64_t addr = qembs_addrs[i][j][l];
                    kernel->getChanRankBankgroupAddress(addr, chan, rank, bg, bank, row, col);
                    input_rows.push_back(row); // inputs rows activates PIM blocks
                    // cout << "PIM input on channel # " << chan << endl;
                }

                    // rd_addr = executePIMMultipleQembs(pooling_idx, chan, rank, bg, bank, input_rows, col, qembs_addrs[i].size()); // PIM result is stored in rd_addr
                    // hbm_callback.register_read_addr(rd_addr);
                    // if(qembs_addrs[i][j].size() == 0)
                    //     cout << rd_addr << " channel : " << chan << endl;
                    // else
                    //     cout << rd_addr << " else channel : " << chan << endl;

                    // cout << "input rows : " << input_rows.size() << " qembs addr size : " << qembs_addrs[i][j].size() << endl;
                    // // kernel->getChanRankBankgroupAddress(rd_addr, chan, rank, bg, bank, row, col);
                    // // cout << "PIM result on channel # " << chan << " bank # "<< bank << endl;

                if(qembs_addrs[i][j].size() > 0)
                {
                    rd_addr = executePIMMultipleQembs(pooling_idx, chan, rank, bg, bank, input_rows, col, qembs_addrs[i][j].size()); // PIM result is stored in rd_addr
                    hbm_callback.register_read_addr(rd_addr);

                    // ////////////// For Test ////////////////
                    // unsigned chan, rank, bg, bank, row, col;
                    // kernel->getChanRankBankgroupAddress(rd_addr, chan, rank, bg, bank, row, col);
                    // cout << "PIM result on channel # " << chan << " bank # "<< bank << endl;
                }
            }
        }
        for(int k=0;k<DIMM_transaction[pooling_idx].size();k++)
        {
            // cout << "transaction on DIMM " << k << "th out of " << DIMM_transaction[pooling_idx].size() << "  " << DIMM_transaction[pooling_idx][k] << endl;
            ddr4_mem->addTransaction(is_write, DIMM_transaction[pooling_idx][k]);
            ddr4_callback.register_read_addr(DIMM_transaction[pooling_idx][k]);
        }

        return total_used_banks;
    }

    void executeNMP()
    {
        bool is_write = false;
        BurstType nullBst;
        int total_embedding = HBM_transaction.size();
        int add_cycle = 3;
        int hbm_clk = 2;
        int ddr4_clk = 4;
        int ddr4_count = 0;

        for(int i=0; i<100; i++)
        {
//            if(i%100 == 0)
            cout << "processing " << i << " out of " << total_embedding << endl;

            int total_used_banks_in_HBM = addTransactionPerPoolingMultipleQembsWithRespectToChannel(i, is_write);
            int pooling_count = total_used_banks_in_HBM + DIMM_transaction[i].size();
            bool is_calculating = false;
            int nmp_cycle_left = add_cycle;
            int buffer_queue = 0;
            int nmp_cycle = 0;
            int hbm_complete = total_used_banks_in_HBM;
            int dimm_complete = DIMM_transaction[i].size();

            while (pooling_count > 0 || buffer_queue > 0 || nmp_cycle_left > 0)
            {   
                nmp_cycle++;
                hbm_mem->update();
                cur_cycle++;
                ddr4_count++;
                if (ddr4_count == ddr4_clk/hbm_clk)
                {
                    ddr4_mem->update();
                    ddr4_count = 0;
                }

                if(is_calculating)
                {
                    nmp_cycle_left--;
                    if(nmp_cycle_left == 0)
                        is_calculating = false;
                }

                if (buffer_queue > 0 && !is_calculating)
                {
                    buffer_queue--;
                    is_calculating = true;
                    nmp_cycle_left = add_cycle;
                }

                for(int i=0; i<hbm_callback.complete_addr_count; i++)
                {
                    pooling_count--;
                    if(is_calculating)
                        buffer_queue++;
                    else
                    {
                        is_calculating = true;
                        nmp_cycle_left = add_cycle;
                    }

                    // cout << "HBM " << hbm_callback.complete_channel[i] << " " << pooling_count << " " << buffer_queue << " " << nmp_cycle_left << " " << " " << nmp_cycle << endl;
                    hbm_complete--;
                }

                for (int i=0; i<ddr4_callback.complete_addr_count; i++)
                {
                    pooling_count--;
                    if(is_calculating)
                        buffer_queue++;
                    else
                    {
                        is_calculating = true;
                        nmp_cycle_left = add_cycle;
                    }

                    // cout << "DIMM " << ddr4_callback.complete_channel[i] << " "  << pooling_count << " " << buffer_queue << " " << nmp_cycle_left << " " << " " << nmp_cycle << endl;
                    dimm_complete--;
                }
                hbm_callback.complete_addr_check_complete();
                ddr4_callback.complete_addr_check_complete();
            }
        }

        cout << "total cycle : " << cur_cycle << endl;
    }

    void getMemTraffic(string filename)
    {
        string line, str_tmp;
        std::ifstream file(filename);
        std::stringstream ss;
        int count = 0;

        std::vector<uint64_t> HBM_pooling;
        std::vector<uint64_t> DIMM_pooling;
        HBM_transaction.push_back(HBM_pooling);
        DIMM_transaction.push_back(DIMM_pooling);
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
                    std::vector<uint64_t> HBM_pooling;
                    std::vector<uint64_t> DIMM_pooling;
                    HBM_transaction.push_back(HBM_pooling);
                    DIMM_transaction.push_back(DIMM_pooling);
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

        }

        cout << "Done loading trace file" << endl;
    }

        
    private:
        shared_ptr<MultiChannelMemorySystem> hbm_mem;
        shared_ptr<MultiChannelMemorySystem> ddr4_mem;
        shared_ptr<PIMKernelChannelwise> kernel;
        KernelType ktype;

        bool write_;
        uint64_t cur_cycle = 0;
        uint64_t mem_size;
        uint64_t data_size_in_byte;
        uint64_t basic_stride;
        uint64_t vec_size_in_byte;
        MyCallBack hbm_callback;
        MyCallBack ddr4_callback;

        vector <vector<uint64_t>> HBM_transaction;
        vector <vector<uint64_t>> DIMM_transaction;
        vector<vector<int>> qembs_per_bank;
        vector<vector<vector<uint64_t>>> qembs_addrs;
        // vector<vector<int>> total_reduced_vecs_per_bank;
        int total_channels;
        int total_banks;

};

    // int addTransactionPerPoolingMultipleQembs(int pooling_idx, bool is_write)
    // {
    //     int total_used_banks = determineQembsPerBank(pooling_idx);
        
    //     // TODO : add transaction with respect to channel
    //     // add transaction with respect to bank
    //     for(int j=0;j<total_banks;j++)
    //     {   
    //         uint64_t rd_addr;
    //         vector<int> input_rows;
    //         unsigned chan, rank, bg, bank, row, col;

    //         for(int l=0; l<qembs_addrs[j].size(); l++)
    //         {
    //             uint64_t addr = qembs_addrs[j][l];
    //             kernel->getChanRankBankgroupAddress(addr, chan, rank, bg, bank, row, col);
    //             input_rows.push_back(row); // inputs rows activates PIM blocks
    //         }

    //         rd_addr = executePIMMultipleQembs(pooling_idx, chan, rank, bg, bank, input_rows, col, qembs_addrs[j].size()); // PIM result is stored in rd_addr
    //         hbm_callback.register_read_addr(rd_addr);
    //     }

    //     for(int k=0;k<DIMM_transaction[pooling_idx].size();k++)
    //     {
    //         // cout << "transaction on DIMM " << k << "th out of " << DIMM_transaction[pooling_idx].size() << "  " << DIMM_transaction[pooling_idx][k] << endl;
    //         ddr4_mem->addTransaction(is_write, DIMM_transaction[pooling_idx][k]);
    //         ddr4_callback.register_read_addr(DIMM_transaction[pooling_idx][k]);
    //     }
    //     return total_used_banks;
    // }

    // int determineQembsPerBank(int pooling_idx)
    // {
    //     int total_used_banks = 0;
        
    //     qembs_per_bank.clear();
    //     qembs_addrs.clear();

    //     for(int i=0; i<total_banks; i++)
    //     {
    //         qembs_per_bank.push_back(0);
    //         std::vector<uint64_t> dummy;
    //         qembs_addrs.push_back(dummy);
    //     }

    //     for(int j=0;j<HBM_transaction[pooling_idx].size();j++)
    //     {
    //         uint64_t addr = HBM_transaction[pooling_idx][j];
    //         unsigned chan, rank, bg, bank, row, col;
    //         kernel->getChanRankBankgroupAddress(addr, chan, rank, bg, bank, row, col);
    //         qembs_per_bank[bank] = qembs_per_bank[bank] + 1;
    //         qembs_addrs[bank].push_back(addr);
    //     }

    //     // check total banks used in this pooling operation
    //     for(int i=0; i<total_banks; i++)
    //     {
    //         if(qembs_addrs[i].size() > 1)
    //             total_used_banks++;
    //     }

    // //     /////////////// verify the results ///////////////
    // //     /*
    // //     int total_q = 0;
    // //     cout << "pooling idx : " << pooling_idx << " total q's " << HBM_transaction[pooling_idx].size() << endl;
    // //     for(int i=0; i<total_banks; i++)
    // //     {
    // //         total_q += qembs_addrs[i].size();
    // //         cout << "bank " << i << " " << qembs_addrs[i].size() << endl;
    // //     }
    // //     cout << "total q : " << total_q << endl;

    // //     if(i == 5)
    // //         exit(0);
    // //     */

    // //    //////////////////// check reduced vectors ////////////////////

    // //     if(pooling_idx < 100)
    // //     {
    // //         cout << "total vecs : " << HBM_transaction[pooling_idx].size() << endl;
    // //         int total_reduced_vecs = 0;
    // //         for(int i=0; i<total_banks; i++)
    // //         {
    // //                 total_reduced_vecs += qembs_per_bank[i];
    // //                 if(i%2 == 1)
    // //                 {
    // //                     total_reduced_vecs_per_bank[i] = total_reduced_vecs_per_bank[i] + total_reduced_vecs;
    // //                     total_reduced_vecs = 0;
    // //                 }
    // //         }

    // //     }
    // //     if(pooling_idx == 100)
    // //     {
    // //         for(int i=0; i<total_banks; i++)
    // //         {
    // //             total_reduced_vecs_per_bank[i] = total_reduced_vecs_per_bank[i] / 100;
    // //             if(total_reduced_vecs_per_bank[i] % 100 > 50)
    // //             {
    // //                 total_reduced_vecs_per_bank[i] = total_reduced_vecs_per_bank[i] + 1;
    // //             }
    // //             if(i%2 == 1)
    // //             {
    // //                 int total_reduced_vecs_per_pim = total_reduced_vecs_per_bank[i-1] + total_reduced_vecs_per_bank[i];
    // //                 cout << "total reduced vecs on pim block # " << i/2 << " : " << total_reduced_vecs_per_pim << endl;
    // //             }
    // //         }
    // //     }


    //     return total_used_banks;
    // }


class PIMKernelFixture : public testing::Test
{
  public:
    PIMKernelFixture() {}
    ~PIMKernelFixture() {}

    virtual void SetUp()
    {
        printTestMessage();
    }

    virtual void TearDown()
    {
        printResult();
        printFailVector();
    }

    BurstType* getResultPIM(KernelType kn_type, DataDim* dim_data, shared_ptr<PIMKernel> kernel,
                            BurstType* result)
    {
        switch (kn_type)
        {
            case KernelType::GEMV:
            {
                kernel->preloadGemv(&dim_data->weight_npbst_);
                kernel->executeGemv(&dim_data->weight_npbst_, &dim_data->input_npbst_, false);
                unsigned end_col = kernel->getResultColGemv(
                    dim_data->dimTobShape(dim_data->input_dim_), dim_data->output_dim_);
                result = new BurstType[dim_data->output_dim_ * dim_data->batch_size_];
                kernel->readResult(result, pimBankType::ODD_BANK,
                                   dim_data->output_dim_ * dim_data->batch_size_, 0, 0, end_col);
                break;
            }
            case KernelType::ADD:
            case KernelType::MUL:
            {
                int input_row0 = 0;
                int input_row1 = 128;
                int result_row = 256;
                kernel->preloadNoReplacement(&dim_data->input_npbst_, input_row0, 0);
                kernel->preloadNoReplacement(&dim_data->input1_npbst_, input_row1, 0);
                kernel->executeEltwise(dim_data->dimTobShape(dim_data->output_dim_),
                                       pimBankType::ALL_BANK, kn_type, input_row0, result_row,
                                       input_row1);
                result = new BurstType[dim_data->output_dim_];
                kernel->readData(result, dim_data->dimTobShape(dim_data->output_dim_), result_row,
                                 0);
                break;
            }
            case KernelType::RELU:
            {
                int input_row0 = 0;
                int result_row = 256;
                kernel->preloadNoReplacement(&dim_data->input_npbst_, input_row0, 0);
                kernel->executeEltwise(dim_data->dimTobShape(dim_data->output_dim_),
                                       pimBankType::ALL_BANK, kn_type, input_row0, result_row);
                result = new BurstType[dim_data->output_dim_];
                kernel->readData(result, dim_data->dimTobShape(dim_data->output_dim_), result_row,
                                 0);
                break;
            }
            case KernelType::GEMVTREE:
            default:
            {
                ERROR("== Error - Unknown KernelType trying to run");
                break;
            }
        }
        kernel->runPIM();
        return result;
    }

    void expectAccuracy(KernelType kn_type, int num_tests, NumpyBurstType precalculated_result,
                        uint32_t stride = 16)
    {
        switch (kn_type)
        {
            case KernelType::GEMV:
            {
                for (int i = 0; i < num_tests; i++)
                {
                    EXPECT_FP16_EQ(result_[i].fp16ReduceSum(),
                                   precalculated_result.getBurst(0).fp16Data_[i]);
                    reduced_result_[i / stride].fp16Data_[i % stride] = result_[i].fp16ReduceSum();
                }
                return;
            }
            case KernelType::ADD:
            case KernelType::MUL:
            case KernelType::RELU:
            {
                for (int i = 0; i < num_tests; i++)
                {
                    EXPECT_FP16_BST_EQ(result_[i], precalculated_result.getBurst(i));
                }
                return;
            }
            default:
            {
                ERROR("== Error - Unknown KernelType trying to run");
                return;
            }
        }
    }

    shared_ptr<PIMKernel> make_pim_kernel()
    {
        shared_ptr<MultiChannelMemorySystem> mem = make_shared<MultiChannelMemorySystem>(
            "ini/HBM2_samsung_2M_16B_x64.ini", "system_hbm_64ch.ini", ".", "example_app",
            256 * 64 * 2);
        int numPIMChan = 64;
        int numPIMRank = 1;
        shared_ptr<PIMKernel> kernel = make_shared<PIMKernel>(mem, numPIMChan, numPIMRank);

        return kernel;
    }

    /* result data */
    BurstType* result_;
    BurstType* reduced_result_;

    /* stats */
    void testStatsClear()
    {
        testStats.clear();
    }

    void printTestMessage()
    {
        cout << ">>PIM Kernel Accuraccy Test" << endl;
    }

    void printFailVector()
    {
        testStats.printFailVector();
    }

    void printResult()
    {
        cout << endl;
        cout << "> Test Result" << endl;
        cout << "  simulated output comparison via pre-calculated values" << endl;
        cout << "> passed : " << testStats.getNumPassed() << endl;
        cout << "> failed : " << testStats.getNumFailed() << endl;
    }
};

#define GET_NUM_TESTS() \
    testStats.getNumPassed() + testStats.getNumWarning() + testStats.getNumFailed()
#define INC_NUM_PASSED() testStats.IncNumPassed(1)
#define INC_NUM_WARNING() testStats.IncNumWarning(1)
#define INC_NUM_FAILED() testStats.IncNumFailed(1)
#define INSERT_TO_FAILED_VECTOR(var1, var2) \
    testStats.insertToFailVector(GET_NUM_TESTS(), var1, var2)

// A predicate-formatter for asserting that two integers are mutually prime.
::testing::AssertionResult fp16EqualHelper(const char* m_expr, const char* n_expr, fp16 m, fp16 n)
{
    fp16i mi(m);
    fp16i ni(n);
    unsigned cur_idx = GET_NUM_TESTS();
    if (fp16Equal(m, n, 4, 0.01))
    {
        INC_NUM_PASSED();
        return ::testing::AssertionSuccess();
    }
    else if (fp16Equal(m, n, 256, 0.7))
    {
        INC_NUM_PASSED();
        return ::testing::AssertionSuccess();
    }
    else
    {
        INSERT_TO_FAILED_VECTOR(convertH2F(m), convertH2F(n));
        INC_NUM_FAILED();
        return ::testing::AssertionFailure()
               << cur_idx << m_expr << " and " << n_expr << " (" << convertH2F(m) << " and "
               << convertH2F(n) << ") are not same " << mi.ival << " " << ni.ival;
    }
}

::testing::AssertionResult fp16BstEqualHelper(const char* m_expr, const char* n_expr, BurstType mb,
                                              BurstType nb)
{
    for (int i = 0; i < 16; i++)
    {
        fp16 m = mb.fp16Data_[i];
        fp16 n = nb.fp16Data_[i];
        fp16i mi(m);
        fp16i ni(n);

        if (fp16Equal(m, n, 4, 0.01))
        {
            INC_NUM_PASSED();
        }
        else if (fp16Equal(m, n, 256, 0.7))
        {
            INC_NUM_PASSED();
        }
        else
        {
            INC_NUM_FAILED();
            INSERT_TO_FAILED_VECTOR(convertH2F(m), convertH2F(n));
            return ::testing::AssertionFailure()
                   << m_expr << " and " << n_expr << " (" << convertH2F(m) << " and "
                   << convertH2F(n) << ") are not same " << mi.ival << " " << ni.ival;
        }
    }

    return ::testing::AssertionSuccess();
}
#endif
