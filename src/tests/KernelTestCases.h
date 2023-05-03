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
            channel = 0;
            complete_cycle = 0;
        }

        void operator()(unsigned id, uint64_t addr, uint64_t cycle)
        {
            channel = id;
            if (complete_cycle != cycle)
                complete_addr.clear();

        }

        int channel;
        vector<uint64_t> complete_addr;
        uint64_t complete_cycle;
};

class MyPIMFixture : public testing::Test
{
    public:
        MyPIMFixture() {}
        ~MyPIMFixture() {}
    virtual void SetUp()
    {
        cur_cycle = 0;

        hbm_callback = MyCallBack();
        ddr4_callback = MyCallBack();

        hbm_mem = make_shared<MultiChannelMemorySystem>(
            "ini/HBM2_samsung_2M_16B_x64.ini", "system_hbm_64ch.ini", ".", "example_app",
            256 * 16);
        ddr4_mem = make_shared<MultiChannelMemorySystem>(
            "ini/DDR4_8gb_x8_2666.ini", "system_ddr4_1ch.ini", ".", "example_app",
            8*1024);

        hbm_mem->RegisterCallbacks(&hbm_callback, NULL, NULL);
        ddr4_mem->RegisterCallbacks(&ddr4_callback, NULL, NULL);

        mem_size = (uint64_t)getConfigParam(UINT, "NUM_CHANS") * getConfigParam(UINT, "NUM_RANKS") *
                   getConfigParam(UINT, "NUM_BANK_GROUPS") * getConfigParam(UINT, "NUM_BANKS") *
                   getConfigParam(UINT, "NUM_ROWS") * getConfigParam(UINT, "NUM_COLS");

        basic_stride =
            (getConfigParam(UINT, "JEDEC_DATA_BUS_BITS") * getConfigParam(UINT, "BL") / 8);

        int numPIMChan = 64;
        int numPIMRank = 1;
        vec_size_in_byte = 64;

        kernel = make_shared<PIMKernelChannelwise>(hbm_mem, numPIMChan, numPIMRank);
        ktype = KernelType::EMB;

        getMemTraffic("/home/youngsuk95/PIMSimulator/src/tests/pim_trace.txt");        
    }
    void executePIMPerQemb(int pooling_idx, int ch_idx, int ra_idx, int bg_idx, int bank_idx, int row_idx, int col_idx)
    {
        int input_row = row_idx;
        int result_row = 256;
        kernel->executeEltwise(vec_size_in_byte, pimBankType::ALL_BANK, ktype, ch_idx, ra_idx, bank_idx, input_row, result_row);
        uint64_t addr = kernel->readPIMResult(ch_idx, ra_idx, bg_idx, bank_idx, row_idx, col_idx); //TODO : put parameters

        HBM_read_addrs.push_back(addr);
    }

    int addTransactionPerPooling(int pooling_idx, bool is_write, BurstType bst)
    {
        for(int j=0;j<HBM_transaction[pooling_idx].size();j++)
        {
            uint64_t addr = HBM_transaction[pooling_idx][j];
            unsigned chan, rank, bank, row, col;
            kernel->getChanRankBankgroupAddress(addr, chan, rank, bank, row, col);
            executePIMPerQemb(pooling_idx, chan, rank);
            // assume r vector is fetched from SRF
        }

        for(int k=0;k<DIMM_transaction[pooling_idx].size();k++)
        {
            ddr4_mem->addTransaction(is_write, DIMM_transaction[pooling_idx][k], &bst);
        }
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

        for(int i=0; i<total_embedding; i++)
        {
            int pooling_count = HBM_transaction[i].size() + DIMM_transaction[i].size();
            bool is_calculating = false;
            int nmp_cycle_left = add_cycle;
            int buffer_queue = 0;
            int nmp_cycle = 0;

            addTransactionPerPooling(i, is_write, nullBst);
            while (pooling_count > 0 || buffer_queue > 0 || nmp_cycle_left > 0)
            {   
                nmp_cycle++;
                hbm_mem->update();
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


                for (int i=0; i < hbm_callback.complete_addr.size();i++)
                {
                    cout << "HBM " << hbm_callback.channel << " " << pooling_count << " " << buffer_queue << " " << nmp_cycle_left << " " << " " << nmp_cycle << endl;
                    
                    for(int j=0; j< HBM_read_addrs.size(); j++)
                    {
                        if(hbm_callback.complete_addr[i] == HBM_read_addrs[j])
                        {
                            pooling_count--; // r vector is added inside executePIMPerQemb
                            if(is_calculating)
                                buffer_queue++;
                            else
                                is_calculating = true;
                                nmp_cycle_left = add_cycle;
                        }
                    }
                }
                for (int i=0; i<ddr4_callback.complete_addr.size();i++)
                {
                    cout << "DIMM " << ddr4_callback.channel << " " << pooling_count << " " << buffer_queue << " " << nmp_cycle_left << " " << " " << nmp_cycle << endl;

                    pooling_count--;
                    if(is_calculating)
                        buffer_queue++;
                    else
                        is_calculating = true;
                        nmp_cycle_left = add_cycle;
                }

                hbm_callback.complete_addr.clear();
                ddr4_callback.complete_addr.clear();
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

        }
           
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
        vector <uint64_t> HBM_read_addrs;

};


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
