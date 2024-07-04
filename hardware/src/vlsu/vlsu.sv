// Copyright 2021 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Author: Matheus Cavalcante <matheusd@iis.ee.ethz.ch>
// Description:
// This is Ara's vector load/store unit. It is used exclusively for vector
// loads and vector stores. There are no guarantees regarding concurrency
// and coherence with Ariane's own load/store unit.

module vlsu import ara_pkg::*; import rvv_pkg::*; #(
    parameter  int  unsigned NrLanes = 0,
    parameter  type          vaddr_t = logic,  // Type used to address vector register file elements
    // AXI Interface parameters
    parameter  int  unsigned AxiDataWidth = 0,
    parameter  int  unsigned AxiAddrWidth = 0,
    parameter  type          axi_ar_t     = logic,
    parameter  type          axi_r_t      = logic,
    parameter  type          axi_aw_t     = logic,
    parameter  type          axi_w_t      = logic,
    parameter  type          axi_b_t      = logic,
    parameter  type          axi_req_t    = logic,
    parameter  type          axi_resp_t   = logic,
    // Dependant parameters. DO NOT CHANGE!
    localparam int  unsigned DataWidth    = $bits(elen_t),
    localparam type          strb_t       = logic [DataWidth/8-1:0]
  ) (
    input  logic                    clk_i,
    input  logic                    rst_ni,
  `ifdef ARA_L1_INTF
    // L1 D$ interface
    output dcache_req_i_t [1:0]     l1_dcache_req_o,
    input  dcache_req_o_t [1:0]     l1_dcache_resp_i,
  `else
    // AXI Memory Interface
    output axi_req_t                axi_req_o,
    input  axi_resp_t               axi_resp_i,
  `endif
    output logic                    load_is_inprocessing_o,
  `ifdef ARA_VA
    // Interface with TLB
    output logic                    addrgen_trans_req_o,
    output logic [riscv::VLEN-1:0]  addrgen_trans_vaddr_o,
    output logic                    addrgen_trans_is_store_o,
    input  logic                    addrgen_trans_dtlb_hit_i,
    input  logic [riscv::PPNW-1:0]  addrgen_trans_dtlb_ppn_i,
    input  logic                    addrgen_trans_valid_i,
    input  logic [riscv::PLEN-1:0]  addrgen_trans_paddr_i,
    input  exception_t              addrgen_trans_exception_i,
  `endif
    // Interface with the dispatcher
    input  logic                    core_st_pending_i,
    output logic                    load_complete_o,
    output logic                    store_complete_o,
    output logic                    store_pending_o,
    // Interface with the sequencer
    input  pe_req_t                 pe_req_i,
    input  logic                    pe_req_valid_i,
    input  logic      [NrVInsn-1:0] pe_vinsn_running_i,
    output logic      [1:0]         pe_req_ready_o,         // Load (0) and Store (1) units
    output pe_resp_t  [1:0]         pe_resp_o,              // Load (0) and Store (1) units
    output logic                    addrgen_ack_o,
    output ariane_pkg::exception_t  addrgen_exception_o,
    output vlen_t                   addrgen_exception_vstart_o,
    // Interface with the lanes
    // Store unit operands
    input  elen_t     [NrLanes-1:0] stu_operand_i,
    input  logic      [NrLanes-1:0] stu_operand_valid_i,
    output logic      [NrLanes-1:0] stu_operand_ready_o,
    // Address generation operands
    input  elen_t     [NrLanes-1:0] addrgen_operand_i,
    input  target_fu_e[NrLanes-1:0] addrgen_operand_target_fu_i,
    input  logic      [NrLanes-1:0] addrgen_operand_valid_i,
    output logic                    addrgen_operand_ready_o,
    // Interface with the Mask unit
    input  strb_t     [NrLanes-1:0] mask_i,
    input  logic      [NrLanes-1:0] mask_valid_i,
    output logic                    vldu_mask_ready_o,
    output logic                    vstu_mask_ready_o,
    // Results
    output logic      [NrLanes-1:0] ldu_result_req_o,
    output vid_t      [NrLanes-1:0] ldu_result_id_o,
    output vaddr_t    [NrLanes-1:0] ldu_result_addr_o,
    output elen_t     [NrLanes-1:0] ldu_result_wdata_o,
    output strb_t     [NrLanes-1:0] ldu_result_be_o,
    input  logic      [NrLanes-1:0] ldu_result_gnt_i,
    input  logic      [NrLanes-1:0] ldu_result_final_gnt_i
  );

  logic load_complete, store_complete;
  logic addrgen_exception_load, addrgen_exception_store;
  assign load_complete_o  = load_complete  | addrgen_exception_load;
  assign store_complete_o = store_complete | addrgen_exception_store;

  ///////////////////
  //  Definitions  //
  ///////////////////

  typedef logic [AxiAddrWidth-1:0] axi_addr_t;

`ifdef ARA_L1_INTF
  dcache_req_i_t [1:0] l1_dcache_req_addrgen;
  logic [AxiDataWidth-1:0] dcache_wdata;
  logic [AxiDataWidth/8-1:0] dcache_wbe;
  logic dcache_wvalid;

  logic load_result_queue_full;

  // read port
  assign l1_dcache_req_o[0].address_index = l1_dcache_req_addrgen[0].address_index;
  assign l1_dcache_req_o[0].address_tag = l1_dcache_req_addrgen[0].address_tag;
  assign l1_dcache_req_o[0].data_req = l1_dcache_req_addrgen[0].data_req;
  assign l1_dcache_req_o[0].data_size = l1_dcache_req_addrgen[0].data_size;
  assign l1_dcache_req_o[0].tag_valid = l1_dcache_req_addrgen[0].tag_valid & ~load_result_queue_full;
  assign l1_dcache_req_o[0].data_wdata = 'b0;
  assign l1_dcache_req_o[0].data_we= 1'b0;
  assign l1_dcache_req_o[0].data_be = 'b0;
  assign l1_dcache_req_o[0].kill_req = 1'b0;

  // store port
  assign l1_dcache_req_o[1].address_index = l1_dcache_req_addrgen[1].address_index;
  assign l1_dcache_req_o[1].address_tag = l1_dcache_req_addrgen[1].address_tag;
  assign l1_dcache_req_o[1].data_req = dcache_wvalid;    
  assign l1_dcache_req_o[1].data_size = l1_dcache_req_addrgen[1].data_size;
  assign l1_dcache_req_o[1].tag_valid = dcache_wvalid;
  assign l1_dcache_req_o[1].data_wdata = dcache_wdata;
  assign l1_dcache_req_o[1].data_we= 1'b1;
  assign l1_dcache_req_o[1].data_be = dcache_wbe;
  assign l1_dcache_req_o[1].kill_req = 1'b0;
`else

  ///////////////
  //  AXI Cut  //
  ///////////////

  // Internal AXI request signals
  axi_req_t  axi_req;
  axi_resp_t axi_resp;

  axi_cut #(
    .ar_chan_t (axi_ar_t  ),
    .r_chan_t  (axi_r_t   ),
    .aw_chan_t (axi_aw_t  ),
    .w_chan_t  (axi_w_t   ),
    .b_chan_t  (axi_b_t   ),
    .axi_req_t (axi_req_t ),
    .axi_resp_t(axi_resp_t)
  ) i_axi_cut (
    .clk_i     (clk_i     ),
    .rst_ni    (rst_ni    ),
    .mst_req_o (axi_req_o ),
    .mst_resp_i(axi_resp_i),
    .slv_req_i (axi_req   ),
    .slv_resp_o(axi_resp  )
  );
`endif

  //////////////////////////
  //  Address Generation  //
  //////////////////////////

  // Interface with the load/store units
  addrgen_axi_req_t axi_addrgen_req;
  logic             axi_addrgen_req_valid;
  logic             ldu_axi_addrgen_req_ready;
  logic             stu_axi_addrgen_req_ready;

  addrgen #(
    .NrLanes     (NrLanes     ),
    .AxiDataWidth(AxiDataWidth),
    .AxiAddrWidth(AxiAddrWidth),
    .axi_ar_t    (axi_ar_t    ),
    .axi_aw_t    (axi_aw_t    )
  ) i_addrgen (
    .clk_i                      (clk_i                      ),
    .rst_ni                     (rst_ni                     ),
  `ifdef ARA_L1_INTF
    .l1_dcache_req_o            (l1_dcache_req_addrgen      ),
    .l1_dcache_gnt_i            ({l1_dcache_resp_i[1].data_gnt, l1_dcache_resp_i[0].data_gnt}),
  `else
    // AXI Memory Interface
    .axi_aw_o                   (axi_req.aw                 ),
    .axi_aw_valid_o             (axi_req.aw_valid           ),
    .axi_aw_ready_i             (axi_resp.aw_ready          ),
    .axi_ar_o                   (axi_req.ar                 ),
    .axi_ar_valid_o             (axi_req.ar_valid           ),
    .axi_ar_ready_i             (axi_resp.ar_ready          ),
  `endif
    .load_is_inprocessing_o     (load_is_inprocessing_o     ),
  `ifdef ARA_VA
    .addrgen_trans_req_o        (addrgen_trans_req_o        ),
    .addrgen_trans_vaddr_o      (addrgen_trans_vaddr_o      ),
    .addrgen_trans_is_store_o   (addrgen_trans_is_store_o   ),
    .addrgen_trans_dtlb_hit_i   (addrgen_trans_dtlb_hit_i   ),
    .addrgen_trans_dtlb_ppn_i   (addrgen_trans_dtlb_ppn_i   ),
    .addrgen_trans_valid_i      (addrgen_trans_valid_i      ),
    .addrgen_trans_paddr_i      (addrgen_trans_paddr_i      ),
    .addrgen_trans_exception_i  (addrgen_trans_exception_i  ),  
  `endif
    // Interface with dispatcher
    .core_st_pending_i          (core_st_pending_i          ),
    // Interface with the sequencer
    .pe_req_i                   (pe_req_i                   ),
    .pe_req_valid_i             (pe_req_valid_i             ),
    .pe_vinsn_running_i         (pe_vinsn_running_i         ),
    .addrgen_ack_o              (addrgen_ack_o              ),
    .addrgen_exception_o        ( addrgen_exception_o       ),
    .addrgen_exception_vstart_o ( addrgen_exception_vstart_o),
    .addrgen_exception_load_o   ( addrgen_exception_load    ),
    .addrgen_exception_store_o  ( addrgen_exception_store   ),
    // Interface with the lanes
    .addrgen_operand_i          (addrgen_operand_i          ),
    .addrgen_operand_target_fu_i(addrgen_operand_target_fu_i),
    .addrgen_operand_valid_i    (addrgen_operand_valid_i    ),
    .addrgen_operand_ready_o    (addrgen_operand_ready_o    ),
    // Interface with the load/store units
    .axi_addrgen_req_o          (axi_addrgen_req            ),
    .axi_addrgen_req_valid_o    (axi_addrgen_req_valid      ),
    .ldu_axi_addrgen_req_ready_i(ldu_axi_addrgen_req_ready  ),
    .stu_axi_addrgen_req_ready_i(stu_axi_addrgen_req_ready  )
  );

  ////////////////////////
  //  Vector Load Unit  //
  ////////////////////////

  vldu #(
    .AxiAddrWidth(AxiAddrWidth),
    .AxiDataWidth(AxiDataWidth),
    .axi_r_t     (axi_r_t     ),
    .NrLanes     (NrLanes     ),
    .vaddr_t     (vaddr_t     )
  ) i_vldu (
    .clk_i                  (clk_i                     ),
    .rst_ni                 (rst_ni                    ),
  `ifdef ARA_L1_INTF
    .dcache_rdata_i         (l1_dcache_resp_i[0].data_rdata),
    .dcache_rvalid_i        (l1_dcache_resp_i[0].data_rvalid),
    .result_queue_full_o    (load_result_queue_full    ),
  `else
    // AXI Memory Interface
    .axi_r_i                (axi_resp.r                ),
    .axi_r_valid_i          (axi_resp.r_valid          ),
    .axi_r_ready_o          (axi_req.r_ready           ),
  `endif
    // Interface with the dispatcher
    .load_complete_o        (load_complete             ),
    // Interface with the main sequencer
    .pe_req_i               (pe_req_i                  ),
    .pe_req_valid_i         (pe_req_valid_i            ),
    .pe_vinsn_running_i     (pe_vinsn_running_i        ),
    .pe_req_ready_o         (pe_req_ready_o[OffsetLoad]),
    .pe_resp_o              (pe_resp_o[OffsetLoad]     ),
    // Interface with the address generator
    .axi_addrgen_req_i      (axi_addrgen_req           ),
    .axi_addrgen_req_valid_i(axi_addrgen_req_valid     ),
    .axi_addrgen_req_ready_o(ldu_axi_addrgen_req_ready ),
    // Interface with the Mask unit
    .mask_i                 (mask_i                    ),
    .mask_valid_i           (mask_valid_i              ),
    .mask_ready_o           (vldu_mask_ready_o         ),
    // Interface with the lanes
    .ldu_result_req_o       (ldu_result_req_o          ),
    .ldu_result_addr_o      (ldu_result_addr_o         ),
    .ldu_result_id_o        (ldu_result_id_o           ),
    .ldu_result_wdata_o     (ldu_result_wdata_o        ),
    .ldu_result_be_o        (ldu_result_be_o           ),
    .ldu_result_gnt_i       (ldu_result_gnt_i          ),
    .ldu_result_final_gnt_i (ldu_result_final_gnt_i    )
  );

  /////////////////////////
  //  Vector Store Unit  //
  /////////////////////////

  vstu #(
    .AxiAddrWidth(AxiAddrWidth),
    .AxiDataWidth(AxiDataWidth),
    .axi_w_t     (axi_w_t     ),
    .axi_b_t     (axi_b_t     ),
    .NrLanes     (NrLanes     ),
    .vaddr_t     (vaddr_t     )
  ) i_vstu (
    .clk_i                  (clk_i                      ),
    .rst_ni                 (rst_ni                     ),
  `ifdef ARA_L1_INTF
    .dcache_wdata_o         (dcache_wdata               ),
    .dcache_wbe_o           (dcache_wbe                 ),
    .dcache_wvalid_o        (dcache_wvalid              ),
    .dcache_wgnt_i          (l1_dcache_resp_i[1].data_gnt),
  `else
    // AXI Memory Interface
    .axi_w_o                (axi_req.w                  ),
    .axi_w_valid_o          (axi_req.w_valid            ),
    .axi_w_ready_i          (axi_resp.w_ready           ),
    .axi_b_i                (axi_resp.b                 ),
    .axi_b_valid_i          (axi_resp.b_valid           ),
    .axi_b_ready_o          (axi_req.b_ready            ),
  `endif
    // Interface with the dispatcher
    .store_pending_o        (store_pending_o            ),
    .store_complete_o       (store_complete             ),
    // Interface with the main sequencer
    .pe_req_i               (pe_req_i                   ),
    .pe_req_valid_i         (pe_req_valid_i             ),
    .pe_vinsn_running_i     (pe_vinsn_running_i         ),
    .pe_req_ready_o         (pe_req_ready_o[OffsetStore]),
    .pe_resp_o              (pe_resp_o[OffsetStore]     ),
    // Interface with the address generator
    .axi_addrgen_req_i      (axi_addrgen_req            ),
    .axi_addrgen_req_valid_i(axi_addrgen_req_valid      ),
    .axi_addrgen_req_ready_o(stu_axi_addrgen_req_ready  ),
    // Interface with the Mask unit
    .mask_i                 (mask_i                     ),
    .mask_valid_i           (mask_valid_i               ),
    .mask_ready_o           (vstu_mask_ready_o          ),
    // Interface with the lanes
    .stu_operand_i          (stu_operand_i              ),
    .stu_operand_valid_i    (stu_operand_valid_i        ),
    .stu_operand_ready_o    (stu_operand_ready_o        )
  );

  //////////////////
  //  Assertions  //
  //////////////////

  if (AxiDataWidth == 0)
    $error("[vlsu] The data width of the AXI bus cannot be zero.");

  if (AxiAddrWidth == 0)
    $error("[vlsu] The address width of the AXI bus cannot be zero.");

  if (NrLanes == 0)
    $error("[vlsu] Ara needs to have at least one lane.");

endmodule : vlsu
