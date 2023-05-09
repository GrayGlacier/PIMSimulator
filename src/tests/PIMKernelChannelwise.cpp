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

#include <iomanip>
#include <string>

#include "AddressMapping.h"
#include "tests/PIMCmdGen.h"
#include "tests/PIMKernelChannelwise.h"

void PIMKernelChannelwise::runPIM()
{
    while (mem_->hasPendingTransactions())
    {
        cycle_++;
        mem_->update();
    }
}

uint64_t PIMKernelChannelwise::getCycle()
{
    return cycle_;
}

void PIMKernelChannelwise::parkIn(int ch_idx, int ra_idx)
{
    addBarrier(ch_idx);
    for (int bank_idx = 0; bank_idx < num_banks_ / num_bank_groups_; bank_idx++)
    {
        for (int bg_idx = 0; bg_idx < num_bank_groups_; bg_idx++)
        {
            string str = "PARK_IN_";
            if (bg_idx == 0 && bank_idx == 0)
                str = "START_" + str;
            else if (bg_idx == 3 && bank_idx == 3)
                str = "END_" + str;
            mem_->addTransaction(
                false,
                pim_addr_mgr_->addrGen(ch_idx, ra_idx, bg_idx, bank_idx, (1 << 13), 0), str,
                &null_bst_);
        }
    }
    addBarrier(ch_idx);
}

void PIMKernelChannelwise::parkOut(int ch_idx, int ra_idx)
{
    mem_->addTransaction(false, pim_addr_mgr_->addrGen(ch_idx, ra_idx, 0, 0, (1 << 13), 0),
                            "START_PARK_OUT_", &null_bst_);
    mem_->addTransaction(false, pim_addr_mgr_->addrGen(ch_idx, ra_idx, 0, 1, (1 << 13), 0),
                            "END_PARK_OUT_", &null_bst_);
    addBarrier(ch_idx);
}

void PIMKernelChannelwise::addTransactionAll(bool is_write, int ch_idx, int ra_idx, int bg_idx, int bank_idx, 
                                    int row, int col, const string tag, BurstType* bst, bool use_barrier, int num_loop)
{
    unsigned local_row = row;
    unsigned local_col = col;
    for (int i = 0; i < num_loop; i++)
    {
        uint64_t addr = pim_addr_mgr_->addrGenSafe(ch_idx, ra_idx, bg_idx, bank_idx,
                                                    local_row, local_col);
        (tag != "") ? mem_->addTransaction(is_write, addr, tag, bst)
                    : mem_->addTransaction(is_write, addr, bst);
        local_col++;
    }

    if (use_barrier)
        addBarrier(ch_idx);
}

void PIMKernelChannelwise::addTransactionWithRawAddress(bool is_write, uint64_t addr, int ch_idx, bool use_barrier, 
                                                        BurstType* bst)
{
    mem_->addTransaction(is_write, addr, bst);
    if (use_barrier)
        addBarrier(ch_idx);
}

void PIMKernelChannelwise::getChanRankBankgroupAddress(uint64_t addr, unsigned& chan, unsigned& rank, unsigned& bg,
                                                        unsigned& bank, unsigned& row, unsigned& col)
{
    AddrMapping addrmap;
    addrmap.addressMapping(addr, chan, rank, bank, row, col);
    bg = addrmap.bankgroupId(bank);
}

void PIMKernelChannelwise::addTransactionAll(bool is_write, int ch_idx, int ra_idx, int bg_idx, int bank_idx, 
                                    int row, int col, BurstType* bst, bool use_barrier, int num_loop)
{
    addTransactionAll(is_write, ch_idx, ra_idx, bg_idx, bank_idx, row, col, "", bst, use_barrier, num_loop);
}

void PIMKernelChannelwise::addBarrier(int ch_idx)
{
    mem_->addBarrier(ch_idx);
}

void PIMKernelChannelwise::changePIMMode(dramMode curMode, dramMode nextMode, int ch_idx, int ra_idx)
{
    if (curMode == dramMode::SB && nextMode == dramMode::HAB)
    {
        addTransactionAll(true, ch_idx, ra_idx, 0, 0, 0x27ff, 0x1f, "START_SB_TO_HAB_", &null_bst_);
        addTransactionAll(true, ch_idx, ra_idx, 0, 1, 0x27ff, 0x1f, &null_bst_);
        if (num_banks_ >= 2)
        {
            addTransactionAll(true, ch_idx, ra_idx, 2, 0, 0x27ff, 0x1f, &null_bst_);
            addTransactionAll(true, ch_idx, ra_idx, 2, 1, 0x27ff, 0x1f, "END_SB_TO_HAB_", &null_bst_);
        }
    }
    else if (curMode == dramMode::HAB)
    {
        if (nextMode == dramMode::SB)
        {
            addTransactionAll(true, ch_idx, ra_idx, 0, 0, 0x2fff, 0x1f, "START_HAB_TO_SB", &null_bst_);
            addTransactionAll(true, ch_idx, ra_idx, 0, 1, 0x2fff, 0x1f, "END_HAB_TO_SB", &null_bst_);
        }
        else if (nextMode == dramMode::HAB_PIM)
        {
            addTransactionAll(true, ch_idx, ra_idx, 0, 0, 0x3fff, 0x0, "PIM", &bst_hab_pim_);
        }
    }
    else if (curMode == dramMode::HAB_PIM && nextMode == dramMode::HAB)
        addTransactionAll(true, ch_idx, ra_idx, 0, 0, 0x3fff, 0x0, "PIM", &bst_hab_);

    addBarrier(ch_idx);
}

/*
void PIMKernelChannelwise::preprocessBn(NumpyBurstType* mean_npbst, NumpyBurstType* var_npbst,
                             NumpyBurstType* gamma_npbst, NumpyBurstType* beta_npbst,
                             NumpyBurstType* input_npbst, fp16** params, float eps)
{
    for (int i = 0; i < input_npbst->bShape[0]; i++)
    {
        params[i][0] = 1 / sqrt((float)var_npbst->getBurst(i / 16).fp16Data_[i % 16] + eps);
        params[i][1] = gamma_npbst->getBurst(i / 16).fp16Data_[i % 16];
        params[i][2] = -mean_npbst->getBurst(i / 16).fp16Data_[i % 16] /
                       sqrt((float)var_npbst->getBurst(i / 16).fp16Data_[i % 16] + eps);
        params[i][3] = beta_npbst->getBurst(i / 16).fp16Data_[i % 16];
    }
}

// FIXME : FIX size of srf_bst_. if ch_model is bigger than memory channel, it is not defined.
void PIMKernelChannelwise::preprocessSrf(NumpyBurstType* input_npbst, fp16** params, int burst_offset,
                              int num_srf_usage)
{
    int ch_idx = 0;
    int ra_idx = 0;
    int burst_idx = 0;
    int num_stride_reg = 2;
    srf_bst_ = new BurstType[num_pim_chans_ * num_pim_ranks_];

    for (int ch_model = 0; ch_model < input_npbst->bShape[0]; ch_model++)
    {
        srf_bst_[ch_idx * num_pim_ranks_ + ra_idx].fp16Data_[burst_idx] =
            params[ch_model][0]; // scale
        srf_bst_[ch_idx * num_pim_ranks_ + ra_idx].fp16Data_[burst_idx + 1] =
            params[ch_model][1]; // gamma
        srf_bst_[ch_idx * num_pim_ranks_ + ra_idx].fp16Data_[burst_idx + 8] =
            params[ch_model][2]; // shift
        srf_bst_[ch_idx * num_pim_ranks_ + ra_idx].fp16Data_[burst_idx + 9] =
            params[ch_model][3]; // beta

        ra_idx++;
        if (ra_idx >= num_pim_ranks_)
        {
            ra_idx = 0;
            ch_idx++;
        }
        if (ch_idx >= num_pim_chans_)
        {
            ch_idx = 0;
            burst_idx += num_stride_reg;
        }
        if (burst_idx >= 8)
        {
            cout << "error: this is not defined" <<endl;
        }
    }
}

void PIMKernelChannelwise::programSrf()
{
   for (int ch_idx = 0; ch_idx < num_pim_chans_; ch_idx++)
   {
       for (int ra_idx = 0; ra_idx < num_pim_ranks_; ra_idx++)
       {
           mem_->addTransaction(true, pim_addr_mgr_->addrGen(ch_idx, ra_idx, 0, 0, 0x3fff, 0x1),
           &srf_bst_[ch_idx*num_pim_ranks_ + ra_idx]);
       }
   }
   addBarrier();
}
*/

void PIMKernelChannelwise::programCrf(vector<PIMCmd>& cmds, int ch_idx, int ra_idx)
{
    PIMCmd nop_cmd(PIMCmdType::NOP, 0);
    for (int i = 0; i < 4; i++)
    {
        if (i * 8 >= cmds.size())
            break;
        crf_bst_[i].set(nop_cmd.toInt(), nop_cmd.toInt(), nop_cmd.toInt(), nop_cmd.toInt(),
                        nop_cmd.toInt(), nop_cmd.toInt(), nop_cmd.toInt(), nop_cmd.toInt());
        for (int j = 0; j < 8; j++)
        {
            if (i * 8 + j >= cmds.size())
                break;
            crf_bst_[i].u32Data_[j] = cmds[i * 8 + j].toInt();
        }
        addTransactionAll(true, ch_idx, ra_idx, 0, 1, 0x3fff, 0x4 + i, "PROGRAM_CRF", &(crf_bst_[i]));
    }
    addBarrier(ch_idx);
}

void PIMKernelChannelwise::setCrf(BurstType* bst, bool pim_op, bool use_all_grf, int crf_toggle_cond,
                       bool grfA_zero, bool grfB_zero)
{
    bst->u8Data_[0] = pim_op;
    bst->u8Data_[10] = use_all_grf;
    bst->u8Data_[16] = crf_toggle_cond;
    bst->u8Data_[20] = grfA_zero;
    bst->u8Data_[21] = grfB_zero;
}

unsigned PIMKernelChannelwise::getResultColGemv(int input_dim, int output_dim)
{
    int num_output_tiles = ceil(((double)output_dim / (num_total_pim_blocks_)) / num_grfB_);
    int num_input_tiles = ceil((double)input_dim / (double)num_grfA_);

    return num_output_tiles * num_input_tiles / 2 * num_grfA_ * num_grfB_;
}

void PIMKernelChannelwise::changeBank(pimBankType pb_type, int& ch_idx, int& ra_idx, int& bg_idx,
                           int& bank_idx, unsigned& starting_row, unsigned& starting_col,
                           unsigned& row, unsigned& col)
{
    bank_idx += (pb_type == pimBankType::ALL_BANK) ? 1 : (num_banks_ / num_pim_blocks_);

    if (bank_idx >= (num_banks_ / num_bank_groups_))
    {
        bank_idx = 0;
        if (++bg_idx >= num_bank_groups_)
        {
            bg_idx = 0;
            if (++ra_idx >= num_pim_ranks_)
            {
                ra_idx = 0;
                if (++ch_idx >= num_pim_chans_)
                {
                    ch_idx = 0;
                    starting_row = row;
                    starting_col = col;
                }
            }
        }
    }
}

/*
void PIMKernelChannelwise::preloadEltwise(NumpyBurstType* operand, pimBankType pb_type,
                              unsigned starting_row, unsigned starting_col)
{
   int ch_idx = 0;
   int ra_idx = 0;
   int bg_idx = 0;
   int bank_idx = 0;
   int bank_offset =  (int)pb_type % 2;
   uint64_t addr_op;
   int dim_operand = operand->getTotalDim();

   for (int x=0; x < dim_operand; x+=num_grf_)
   {
       unsigned col = starting_col;
       unsigned row = starting_row;

       for (int grf_idx = 0; grf_idx < num_grf_; grf_idx++)
       {
           addr_op = pim_addr_mgr_->addrGenSafe(ch_idx, ra_idx, bg_idx, bank_idx + bank_offset, row,
                                                col);
           mem_->addTransaction(true, addr_op, &operand->bData[x + grf_idx]);
           col++;
       }
       changeBank(pb_type, ch_idx, ra_idx, bg_idx, bank_idx, starting_row, starting_col, row, col);
   }
}
*/
void PIMKernelChannelwise::readResult(BurstType* resultBst, pimBankType pb_type, int output_dim,
                           uint64_t base_addr, unsigned starting_row, unsigned starting_col)
{
    int ch_idx = 0;
    int ra_idx = 0;
    int bg_idx = 0;
    int bank_idx = 0;
    int bank_offset = (int)pb_type % 2;
    uint64_t addr;

    for (int x = 0; x < output_dim; x += num_grf_)
    {
        unsigned row = starting_row;
        unsigned col = starting_col;

        for (int grf_idx = 0; grf_idx < num_grf_; grf_idx++)
        {
            addr = pim_addr_mgr_->addrGenSafe(ch_idx, ra_idx, bg_idx, bank_idx + bank_offset, row,
                                              col);
            mem_->addTransaction(false, base_addr + addr, "output", &resultBst[x + grf_idx]);
            col++;
        }
        changeBank(pb_type, ch_idx, ra_idx, bg_idx, bank_idx, starting_row, starting_col, row, col);
    }
}

uint64_t PIMKernelChannelwise::readPIMResult(int ch_idx, int ra_idx, int bg_idx, int bank_idx, unsigned row, unsigned col)
{
    uint64_t addr = pim_addr_mgr_->addrGenSafe(ch_idx, ra_idx, bg_idx, bank_idx, row, col);
    mem_->addTransaction(false, addr, "output", &null_bst_);

    return addr;
}


void PIMKernelChannelwise::executeEltwise(int dim, pimBankType pb_type, KernelType ktype, int ch_idx, int ra_idx,
                                 int bank_idx, int input_row, int result_row)
{
    // TODO : num_jump_to_be_taken??
    // how many num_tiles are needed? (dim / num_grf ?? -> dim : byte, num_grf : byte (fp 16) -> how many bytes?)
    // need to consider dim with burst byte and precision byte
    int num_tile = 8;
    int num_jump_to_be_taken = num_tile - 1;
    vector<PIMCmd> pim_cmds = PIMCmdGen::getPIMCmds(ktype, bank_idx, num_jump_to_be_taken, 0, 0);

    int crf_toggle_cond = -1;
    // set Toggle Condition
    switch (pb_type)
    {
        case pimBankType::EVEN_BANK:
            crf_toggle_cond = 2;
            break;
        case pimBankType::ODD_BANK:
            crf_toggle_cond = 1;
            break;
        case pimBankType::ALL_BANK:
            crf_toggle_cond = 0;
            break;
        default:
            crf_toggle_cond = -1;
            break;
    }

    setCrf(&bst_hab_pim_, true, use_all_grf_, crf_toggle_cond, false, false);
    setCrf(&bst_hab_, false, use_all_grf_, crf_toggle_cond, false, false);

    parkIn(ch_idx, ra_idx);
    changePIMMode(dramMode::SB, dramMode::HAB, ch_idx, ra_idx);
    programCrf(pim_cmds, ch_idx, ra_idx);
    changePIMMode(dramMode::HAB, dramMode::HAB_PIM, ch_idx, ra_idx);

    if (ktype == KernelType::EMB)
        computeEmbOp(num_tile, ch_idx, ra_idx, bank_idx, input_row, result_row);

    changePIMMode(dramMode::HAB_PIM, dramMode::HAB, ch_idx, ra_idx);
    changePIMMode(dramMode::HAB, dramMode::SB, ch_idx, ra_idx);
    parkOut(ch_idx, ra_idx);
}

void PIMKernelChannelwise::computeEmbOp(int num_tile, int ch_idx, int ra_idx, int bank_idx, int input_row, int result_row)
{

    // assume r vector is fetched from SRF
    int bank_parity = bank_idx % 2;
    for (int i = 0; i < num_tile; i++)
    {
        int c = num_grf_ * i;
        addTransactionAll(false, ch_idx, ra_idx, 0, bank_parity, 0, 0, "SRF_TO_GRF", &null_bst_, true,
                            num_grf_);
        addTransactionAll(false, ch_idx, ra_idx, 0, bank_parity, input_row, c, "BANK_TO_GRF_AND_ADD", &null_bst_, true, num_grf_);
        addTransactionAll(true, ch_idx, ra_idx, 0, bank_parity, result_row, c, "GRF_TO_BANK", &null_bst_, true, num_grf_);
    }
}

/*
void PIMKernelChannelwise::computeBn(int num_tile, int input0_row, int result_row)
{
    for (int ch_idx = 0; ch_idx < num_pim_chans_; ch_idx++)
    {
        for (int ra_idx = 0; ra_idx < num_pim_ranks_; ra_idx++)
        {
            int srf_bst_num = (input0_row != result_row)? (ch_idx * num_pim_ranks_ + ra_idx) : 0;
            mem_->addTransaction(true, pim_addr_mgr_->addrGen(ch_idx, ra_idx, 0, 0, 0x3fff, 0x1),
                                 &srf_bst_[srf_bst_num]);
        }
    }
    addBarrier();

    if (input0_row != result_row)
        input0_row = result_row = 0;
    for (int i = 0; i < num_tile; i++)
    {
        for (int b = 0; b < 2; b++) // for even/ddd banks, respectively
        {
            addTransactionAll(false, 0, b, input0_row, num_grf_ * i, "MAD1", &null_bst_,
                              true, num_grf_);
            addTransactionAll(false, 0, b, input0_row, num_grf_ * i, "MAD2", &null_bst_,
                              true, num_grf_);
            addTransactionAll(true , 0, b, result_row, num_grf_ * i, "GRF_TO_BANK", &null_bst_,
                              true, num_grf_);
        }
        addTransactionAll(true , 0, 1, result_row, num_grf_ * i + num_grf_ - 1, "GRF_TO_BANK_DUMMY",
                          &null_bst_, true, 1);
    }
}
*/

void PIMKernelChannelwise::readData(BurstType* bst_data, size_t bst_cnt, unsigned starting_row,
                         unsigned starting_col)
{
    uint64_t init_addr = pim_addr_mgr_->addrGenSafe(0, 0, 0, 0, starting_row, starting_col);

    for (uint64_t addr = init_addr, i = 0; i < bst_cnt; addr += transaction_size_, i++)
    {
        mem_->addTransaction(false, addr, &bst_data[i]);
    }
}

void PIMKernelChannelwise::adderTree(BurstType* result, int output_dim, int num_tile, int step, fp16* temp)
{
    if (num_tile == 1)
        return;

    int iter = num_tile / 2;
    if (step == 0)
    {
        for (int i = 0; i < iter; i++)
        {
            temp[i] = result[2 * i * output_dim].fp16AdderTree() +
                      result[(2 * i + 1) * output_dim].fp16AdderTree();
        }
    }
    else
    {
        for (int i = 0; i < iter; i++) temp[i] = temp[i * 2] + temp[i * 2 + 1];

        if (num_tile % 2 == 1)
            temp[iter] = temp[num_tile];
    }

    adderTree(result, output_dim, ceil(double(num_tile) / (double)2), step + 1, temp);

    return;
}
