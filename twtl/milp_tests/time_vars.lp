\ LP format - for model browsing. Use MPS format to capture full model detail.
Minimize
  z_trans_0_0_('q0',_0)_('q0',_2) + 0 zeta_('q0',_0)_('q0',_2)_0
   + z_trans_0_0_('q0',_0)_('q0',_1) + 0 zeta_('q0',_0)_('q0',_1)_0
   + z_trans_0_1_('q0',_0)_('q0',_2) + 0 zeta_('q0',_0)_('q0',_2)_0
   + z_trans_0_1_('q0',_0)_('q0',_1) + 0 zeta_('q0',_0)_('q0',_1)_0
   + z_trans_1_1_('q0',_0)_('q0',_2) + 0 zeta_('q0',_0)_('q0',_2)_0
   + z_trans_1_1_('q0',_0)_('q0',_1) + 0 zeta_('q0',_0)_('q0',_1)_0
   + z_trans_1_2_('q0',_0)_('q0',_2) + 0 zeta_('q0',_0)_('q0',_2)_0
   + z_trans_1_2_('q0',_0)_('q0',_1) + 0 zeta_('q0',_0)_('q0',_1)_0
   + 2 z_trans_1_4_('q0',_0)_('q0',_2) + 0 zeta_('q0',_0)_('q0',_2)_0
   + 2 z_trans_1_4_('q0',_0)_('q0',_1) + 0 zeta_('q0',_0)_('q0',_1)_0
   + z_trans_2_2_('q0',_0)_('q0',_2) + 0 zeta_('q0',_0)_('q0',_2)_0
   + z_trans_2_2_('q0',_0)_('q0',_1) + 0 zeta_('q0',_0)_('q0',_1)_0
   + z_trans_2_3_('q0',_0)_('q0',_2) + 0 zeta_('q0',_0)_('q0',_2)_0
   + z_trans_2_3_('q0',_0)_('q0',_1) + 0 zeta_('q0',_0)_('q0',_1)_0
   + z_trans_3_3_('q0',_0)_('q0',_2) + 0 zeta_('q0',_0)_('q0',_2)_0
   + z_trans_3_3_('q0',_0)_('q0',_1) + 0 zeta_('q0',_0)_('q0',_1)_0
   + z_trans_3_4_('q0',_0)_('q0',_2) + 0 zeta_('q0',_0)_('q0',_2)_0
   + z_trans_3_4_('q0',_0)_('q0',_1) + 0 zeta_('q0',_0)_('q0',_1)_0
   + 0 z_state_4_('q0',_2) + 0 z_state_4_('q0',_0)
   + 0 z_state_virtual_('v',_'v')
Subject To
 initialization: z_trans_virtual_0_('q0',_0)_('q0',_2) = 1
 flow: z_trans_0_1_('q0',_0)_('q0',_2) + z_trans_1_2_('q0',_0)_('q0',_2)
   + z_trans_1_4_('q0',_0)_('q0',_2) + z_trans_2_3_('q0',_0)_('q0',_2)
   + z_trans_3_4_('q0',_0)_('q0',_2)
   + z_trans_virtual_0_('q0',_0)_('q0',_2) = 0
 flow: z_trans_0_1_('q0',_0)_('q0',_1) + z_trans_1_2_('q0',_0)_('q0',_1)
   + z_trans_1_4_('q0',_0)_('q0',_1) + z_trans_2_3_('q0',_0)_('q0',_1)
   + z_trans_3_4_('q0',_0)_('q0',_1)
   + z_trans_virtual_0_('q0',_0)_('q0',_1) = 0
 syncr: z_trans_0_0_('q0',_0)_('q0',_2) + z_trans_0_1_('q0',_0)_('q0',_2)
   + z_trans_1_1_('q0',_0)_('q0',_2) + z_trans_1_2_('q0',_0)_('q0',_2)
   + z_trans_1_4_('q0',_0)_('q0',_2) + z_trans_2_2_('q0',_0)_('q0',_2)
   + z_trans_2_3_('q0',_0)_('q0',_2) + z_trans_3_3_('q0',_0)_('q0',_2)
   + z_trans_3_4_('q0',_0)_('q0',_2)
   + z_trans_virtual_0_('q0',_0)_('q0',_2) - zeta_('q0',_0)_('q0',_2)_0
   <= 0
 syncr: z_trans_0_0_('q0',_0)_('q0',_1) + z_trans_0_1_('q0',_0)_('q0',_1)
   + z_trans_1_1_('q0',_0)_('q0',_1) + z_trans_1_2_('q0',_0)_('q0',_1)
   + z_trans_1_4_('q0',_0)_('q0',_1) + z_trans_2_2_('q0',_0)_('q0',_1)
   + z_trans_2_3_('q0',_0)_('q0',_1) + z_trans_3_3_('q0',_0)_('q0',_1)
   + z_trans_3_4_('q0',_0)_('q0',_1)
   + z_trans_virtual_0_('q0',_0)_('q0',_1) - zeta_('q0',_0)_('q0',_1)_0
   <= 0
 syncr: zeta_('q0',_0)_('q0',_2)_0 + zeta_('q0',_0)_('q0',_1)_0 = 1
 state_transition: z_state_0_('q0',_2) = 0
 state_transition: - z_trans_0_1_('q0',_0)_('q0',_2)
   - z_trans_0_1_('q0',_0)_('q0',_1) + z_state_0_('q0',_0) = 0
 state_transition: z_state_0_('q0',_1) = 0
 state_transition: z_state_0_('v',_'v') = 0
 state_transition: z_state_1_('q0',_2) = 0
 state_transition: - z_trans_1_2_('q0',_0)_('q0',_2)
   - z_trans_1_2_('q0',_0)_('q0',_1) - z_trans_1_4_('q0',_0)_('q0',_2)
   - z_trans_1_4_('q0',_0)_('q0',_1) + z_state_1_('q0',_0) = 0
 state_transition: z_state_1_('q0',_1) = 0
 state_transition: z_state_1_('v',_'v') = 0
 state_transition: z_state_2_('q0',_2) = 0
 state_transition: - z_trans_2_3_('q0',_0)_('q0',_2)
   - z_trans_2_3_('q0',_0)_('q0',_1) + z_state_2_('q0',_0) = 0
 state_transition: z_state_2_('q0',_1) = 0
 state_transition: z_state_2_('v',_'v') = 0
 state_transition: z_state_3_('q0',_2) = 0
 state_transition: - z_trans_3_4_('q0',_0)_('q0',_2)
   - z_trans_3_4_('q0',_0)_('q0',_1) + z_state_3_('q0',_0) = 0
 state_transition: z_state_3_('q0',_1) = 0
 state_transition: z_state_3_('v',_'v') = 0
 state_transition: z_state_4_('v',_'v') = 0
 state_transition: z_state_virtual_('q0',_2) = 0
 state_transition: - z_trans_virtual_0_('q0',_0)_('q0',_2)
   - z_trans_virtual_0_('q0',_0)_('q0',_1) + z_state_virtual_('q0',_0) = 0
 state_transition: z_state_virtual_('q0',_1) = 0
 guard_sat_ts: - z_state_1_('q0',_1) + zeta_('q0',_1)_A <= 0
 guard_sat_product: zeta_('q0',_0)_('q0',_1)_0 - zeta_('q0',_1)_A <= 0
 guard_sat_product: z_state_0_('q0',_1) + z_state_1_('q0',_1)
   + z_state_2_('q0',_1) + z_state_3_('q0',_1) + z_state_4_('q0',_1)
   + z_state_virtual_('q0',_1) - zeta_('q0',_1)_A <= 0
 final: >= 1
Bounds
Binaries
 zeta_('q0',_0)_('q0',_2)_0 zeta_('q0',_0)_('q0',_1)_0
 zeta_('q0',_0)_('q0',_2)_0 zeta_('q0',_0)_('q0',_1)_0
 zeta_('q0',_0)_('q0',_2)_0 zeta_('q0',_0)_('q0',_1)_0
 zeta_('q0',_0)_('q0',_2)_0 zeta_('q0',_0)_('q0',_1)_0
 zeta_('q0',_0)_('q0',_2)_0 zeta_('q0',_0)_('q0',_1)_0
 zeta_('q0',_0)_('q0',_2)_0 zeta_('q0',_0)_('q0',_1)_0
 zeta_('q0',_0)_('q0',_2)_0 zeta_('q0',_0)_('q0',_1)_0
 zeta_('q0',_0)_('q0',_2)_0 zeta_('q0',_0)_('q0',_1)_0
 zeta_('q0',_0)_('q0',_2)_0 zeta_('q0',_0)_('q0',_1)_0
 zeta_('q0',_0)_('q0',_2)_0 zeta_('q0',_0)_('q0',_1)_0 zeta_('q0',_1)_A
Generals
 z_trans_0_0_('q0',_0)_('q0',_2) z_trans_0_0_('q0',_0)_('q0',_1)
 z_trans_0_1_('q0',_0)_('q0',_2) z_trans_0_1_('q0',_0)_('q0',_1)
 z_trans_1_1_('q0',_0)_('q0',_2) z_trans_1_1_('q0',_0)_('q0',_1)
 z_trans_1_2_('q0',_0)_('q0',_2) z_trans_1_2_('q0',_0)_('q0',_1)
 z_trans_1_4_('q0',_0)_('q0',_2) z_trans_1_4_('q0',_0)_('q0',_1)
 z_trans_2_2_('q0',_0)_('q0',_2) z_trans_2_2_('q0',_0)_('q0',_1)
 z_trans_2_3_('q0',_0)_('q0',_2) z_trans_2_3_('q0',_0)_('q0',_1)
 z_trans_3_3_('q0',_0)_('q0',_2) z_trans_3_3_('q0',_0)_('q0',_1)
 z_trans_3_4_('q0',_0)_('q0',_2) z_trans_3_4_('q0',_0)_('q0',_1)
 z_trans_virtual_0_('q0',_0)_('q0',_2)
 z_trans_virtual_0_('q0',_0)_('q0',_1) z_state_0_('q0',_2)
 z_state_0_('q0',_0) z_state_0_('q0',_1) z_state_0_('v',_'v')
 z_state_1_('q0',_2) z_state_1_('q0',_0) z_state_1_('q0',_1)
 z_state_1_('v',_'v') z_state_2_('q0',_2) z_state_2_('q0',_0)
 z_state_2_('q0',_1) z_state_2_('v',_'v') z_state_3_('q0',_2)
 z_state_3_('q0',_0) z_state_3_('q0',_1) z_state_3_('v',_'v')
 z_state_4_('q0',_2) z_state_4_('q0',_0) z_state_4_('q0',_1)
 z_state_4_('v',_'v') z_state_virtual_('q0',_2) z_state_virtual_('q0',_0)
 z_state_virtual_('q0',_1) z_state_virtual_('v',_'v')
End
