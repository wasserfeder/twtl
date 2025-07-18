\ LP format - for model browsing. Use MPS format to capture full model detail.
Minimize
  z_trans_0_0_('q0',_2)_('q0',_3) + 0 zeta_('q0',_2)_('q0',_3)
   + z_trans_0_0_('q0',_0)_('q0',_2) + 0 zeta_('q0',_0)_('q0',_2)
   + z_trans_0_0_('q0',_0)_('q0',_1) + 0 zeta_('q0',_0)_('q0',_1)
   + z_trans_0_0_('q0',_3)_('q0',_4) + 0 zeta_('q0',_3)_('q0',_4)
   + z_trans_0_0_('q0',_1)_('q0',_4) + 0 zeta_('q0',_1)_('q0',_4)
   + z_trans_0_1_('q0',_2)_('q0',_3) + 0 zeta_('q0',_2)_('q0',_3)
   + z_trans_0_1_('q0',_0)_('q0',_2) + 0 zeta_('q0',_0)_('q0',_2)
   + z_trans_0_1_('q0',_0)_('q0',_1) + 0 zeta_('q0',_0)_('q0',_1)
   + z_trans_0_1_('q0',_3)_('q0',_4) + 0 zeta_('q0',_3)_('q0',_4)
   + z_trans_0_1_('q0',_1)_('q0',_4) + 0 zeta_('q0',_1)_('q0',_4)
   + z_trans_1_1_('q0',_2)_('q0',_3) + 0 zeta_('q0',_2)_('q0',_3)
   + z_trans_1_1_('q0',_0)_('q0',_2) + 0 zeta_('q0',_0)_('q0',_2)
   + z_trans_1_1_('q0',_0)_('q0',_1) + 0 zeta_('q0',_0)_('q0',_1)
   + z_trans_1_1_('q0',_3)_('q0',_4) + 0 zeta_('q0',_3)_('q0',_4)
   + z_trans_1_1_('q0',_1)_('q0',_4) + 0 zeta_('q0',_1)_('q0',_4)
   + 0 z_state_2_('q0',_2) + 0 z_state_2_('q0',_0) + 0 z_state_2_('q0',_3)
   + 0 z_state_2_('q0',_1)
Subject To
 initialization: z_trans_virtual_0_('q0',_0)_('q0',_2) = 1
 flow: - z_trans_0_0_('q0',_2)_('q0',_3) + z_trans_0_0_('q0',_0)_('q0',_2)
   - z_trans_0_1_('q0',_2)_('q0',_3) + z_trans_0_1_('q0',_0)_('q0',_2)
   - z_trans_1_1_('q0',_2)_('q0',_3) + z_trans_1_1_('q0',_0)_('q0',_2)
   - z_trans_virtual_0_('q0',_2)_('q0',_3)
   + z_trans_virtual_0_('q0',_0)_('q0',_2) = 0
 flow: z_trans_0_0_('q0',_2)_('q0',_3) - z_trans_0_0_('q0',_3)_('q0',_4)
   + z_trans_0_1_('q0',_2)_('q0',_3) - z_trans_0_1_('q0',_3)_('q0',_4)
   + z_trans_1_1_('q0',_2)_('q0',_3) - z_trans_1_1_('q0',_3)_('q0',_4)
   + z_trans_virtual_0_('q0',_2)_('q0',_3)
   - z_trans_virtual_0_('q0',_3)_('q0',_4) = 0
 flow: z_trans_0_0_('q0',_0)_('q0',_1) - z_trans_0_0_('q0',_1)_('q0',_4)
   + z_trans_0_1_('q0',_0)_('q0',_1) - z_trans_0_1_('q0',_1)_('q0',_4)
   + z_trans_1_1_('q0',_0)_('q0',_1) - z_trans_1_1_('q0',_1)_('q0',_4)
   + z_trans_virtual_0_('q0',_0)_('q0',_1)
   - z_trans_virtual_0_('q0',_1)_('q0',_4) = 0
 syncr: z_trans_0_0_('q0',_2)_('q0',_3) + z_trans_0_1_('q0',_2)_('q0',_3)
   + z_trans_1_1_('q0',_2)_('q0',_3)
   + z_trans_virtual_0_('q0',_2)_('q0',_3) - zeta_('q0',_2)_('q0',_3) <= 0
 syncr: zeta_('q0',_2)_('q0',_3) = 1
 syncr: z_trans_0_0_('q0',_0)_('q0',_2) + z_trans_0_1_('q0',_0)_('q0',_2)
   + z_trans_1_1_('q0',_0)_('q0',_2)
   + z_trans_virtual_0_('q0',_0)_('q0',_2) - zeta_('q0',_0)_('q0',_2) <= 0
 syncr: zeta_('q0',_0)_('q0',_2) + zeta_('q0',_0)_('q0',_1) = 1
 syncr: z_trans_0_0_('q0',_0)_('q0',_1) + z_trans_0_1_('q0',_0)_('q0',_1)
   + z_trans_1_1_('q0',_0)_('q0',_1)
   + z_trans_virtual_0_('q0',_0)_('q0',_1) - zeta_('q0',_0)_('q0',_1) <= 0
 syncr: zeta_('q0',_0)_('q0',_2) + zeta_('q0',_0)_('q0',_1) = 1
 syncr: z_trans_0_0_('q0',_3)_('q0',_4) + z_trans_0_1_('q0',_3)_('q0',_4)
   + z_trans_1_1_('q0',_3)_('q0',_4)
   + z_trans_virtual_0_('q0',_3)_('q0',_4) - zeta_('q0',_3)_('q0',_4) <= 0
 syncr: zeta_('q0',_3)_('q0',_4) = 1
 syncr: z_trans_0_0_('q0',_1)_('q0',_4) + z_trans_0_1_('q0',_1)_('q0',_4)
   + z_trans_1_1_('q0',_1)_('q0',_4)
   + z_trans_virtual_0_('q0',_1)_('q0',_4) - zeta_('q0',_1)_('q0',_4) <= 0
 syncr: zeta_('q0',_1)_('q0',_4) = 1
 state_transition: - z_trans_0_0_('q0',_2)_('q0',_3)
   - z_trans_0_1_('q0',_2)_('q0',_3) + z_state_0_('q0',_2) = 0
 state_transition: - z_trans_0_0_('q0',_0)_('q0',_2)
   - z_trans_0_0_('q0',_0)_('q0',_1) - z_trans_0_1_('q0',_0)_('q0',_2)
   - z_trans_0_1_('q0',_0)_('q0',_1) + z_state_0_('q0',_0) = 0
 state_transition: - z_trans_0_0_('q0',_3)_('q0',_4)
   - z_trans_0_1_('q0',_3)_('q0',_4) + z_state_0_('q0',_3) = 0
 state_transition: - z_trans_0_0_('q0',_1)_('q0',_4)
   - z_trans_0_1_('q0',_1)_('q0',_4) + z_state_0_('q0',_1) = 0
 state_transition: - z_trans_0_0_('q0',_3)_('q0',_4)
   - z_trans_0_0_('q0',_1)_('q0',_4)
   - z_trans_virtual_0_('q0',_3)_('q0',_4)
   - z_trans_virtual_0_('q0',_1)_('q0',_4) + z_state_0_('q0',_4) = 0
 state_transition: - z_trans_1_1_('q0',_2)_('q0',_3) + z_state_1_('q0',_2)
   = 0
 state_transition: - z_trans_1_1_('q0',_0)_('q0',_2)
   - z_trans_1_1_('q0',_0)_('q0',_1) + z_state_1_('q0',_0) = 0
 state_transition: - z_trans_1_1_('q0',_3)_('q0',_4) + z_state_1_('q0',_3)
   = 0
 state_transition: - z_trans_1_1_('q0',_1)_('q0',_4) + z_state_1_('q0',_1)
   = 0
 state_transition: - z_trans_0_1_('q0',_3)_('q0',_4)
   - z_trans_0_1_('q0',_1)_('q0',_4) - z_trans_1_1_('q0',_3)_('q0',_4)
   - z_trans_1_1_('q0',_1)_('q0',_4) + z_state_1_('q0',_4) = 0
 state_transition: - z_trans_virtual_0_('q0',_2)_('q0',_3)
   + z_state_virtual_('q0',_2) = 0
 state_transition: - z_trans_virtual_0_('q0',_0)_('q0',_2)
   - z_trans_virtual_0_('q0',_0)_('q0',_1) + z_state_virtual_('q0',_0) = 0
 state_transition: - z_trans_virtual_0_('q0',_3)_('q0',_4)
   + z_state_virtual_('q0',_3) = 0
 state_transition: - z_trans_virtual_0_('q0',_1)_('q0',_4)
   + z_state_virtual_('q0',_1) = 0
 guard_satisfaction: - z_state_1_('q0',_3) + zeta_('q0',_3)_A <= 0
 product_state_and_term: zeta_('q0',_2)_('q0',_3) - zeta_('q0',_3)_A <= 0
 guard_satisfaction: - z_state_1_('q0',_3) + zeta_('q0',_3)_A <= 0
 product_state_and_term: zeta_('q0',_2)_('q0',_3) - zeta_('q0',_3)_A <= 0
 guard_satisfaction: - z_state_1_('q0',_1) + zeta_('q0',_1)_A <= 0
 product_state_and_term: zeta_('q0',_0)_('q0',_1) - zeta_('q0',_1)_A <= 0
 guard_satisfaction: - z_state_1_('q0',_1) + zeta_('q0',_1)_A <= 0
 product_state_and_term: zeta_('q0',_0)_('q0',_1) - zeta_('q0',_1)_A <= 0
 guard_satisfaction: - z_state_1_('q0',_4) + zeta_('q0',_4)_A <= 0
 product_state_and_term: zeta_('q0',_3)_('q0',_4) - zeta_('q0',_4)_A <= 0
 guard_satisfaction: - z_state_1_('q0',_4) + zeta_('q0',_4)_A <= 0
 product_state_and_term: zeta_('q0',_3)_('q0',_4) - zeta_('q0',_4)_A <= 0
 guard_satisfaction: - z_state_1_('q0',_4) + zeta_('q0',_4)_A <= 0
 product_state_and_term: zeta_('q0',_1)_('q0',_4) - zeta_('q0',_4)_A <= 0
 guard_satisfaction: - z_state_1_('q0',_4) + zeta_('q0',_4)_A <= 0
 product_state_and_term: zeta_('q0',_1)_('q0',_4) - zeta_('q0',_4)_A <= 0
 final: z_state_0_('q0',_4) + z_state_1_('q0',_4) + z_state_2_('q0',_4)
   + z_state_virtual_('q0',_4) >= 1
Bounds
Binaries
 zeta_('q0',_2)_('q0',_3) zeta_('q0',_0)_('q0',_2) zeta_('q0',_0)_('q0',_1)
 zeta_('q0',_3)_('q0',_4) zeta_('q0',_1)_('q0',_4) zeta_('q0',_2)_('q0',_3)
 zeta_('q0',_0)_('q0',_2) zeta_('q0',_0)_('q0',_1) zeta_('q0',_3)_('q0',_4)
 zeta_('q0',_1)_('q0',_4) zeta_('q0',_2)_('q0',_3) zeta_('q0',_0)_('q0',_2)
 zeta_('q0',_0)_('q0',_1) zeta_('q0',_3)_('q0',_4) zeta_('q0',_1)_('q0',_4)
 zeta_('q0',_2)_('q0',_3) zeta_('q0',_0)_('q0',_2) zeta_('q0',_0)_('q0',_1)
 zeta_('q0',_3)_('q0',_4) zeta_('q0',_1)_('q0',_4) zeta_('q0',_3)_A
 zeta_('q0',_3)_A zeta_('q0',_1)_A zeta_('q0',_1)_A zeta_('q0',_4)_A
 zeta_('q0',_4)_A zeta_('q0',_4)_A zeta_('q0',_4)_A
Generals
 z_trans_0_0_('q0',_2)_('q0',_3) z_trans_0_0_('q0',_0)_('q0',_2)
 z_trans_0_0_('q0',_0)_('q0',_1) z_trans_0_0_('q0',_3)_('q0',_4)
 z_trans_0_0_('q0',_1)_('q0',_4) z_trans_0_1_('q0',_2)_('q0',_3)
 z_trans_0_1_('q0',_0)_('q0',_2) z_trans_0_1_('q0',_0)_('q0',_1)
 z_trans_0_1_('q0',_3)_('q0',_4) z_trans_0_1_('q0',_1)_('q0',_4)
 z_trans_1_1_('q0',_2)_('q0',_3) z_trans_1_1_('q0',_0)_('q0',_2)
 z_trans_1_1_('q0',_0)_('q0',_1) z_trans_1_1_('q0',_3)_('q0',_4)
 z_trans_1_1_('q0',_1)_('q0',_4) z_trans_virtual_0_('q0',_2)_('q0',_3)
 z_trans_virtual_0_('q0',_0)_('q0',_2)
 z_trans_virtual_0_('q0',_0)_('q0',_1)
 z_trans_virtual_0_('q0',_3)_('q0',_4)
 z_trans_virtual_0_('q0',_1)_('q0',_4) z_state_0_('q0',_2)
 z_state_0_('q0',_0) z_state_0_('q0',_3) z_state_0_('q0',_1)
 z_state_0_('q0',_4) z_state_1_('q0',_2) z_state_1_('q0',_0)
 z_state_1_('q0',_3) z_state_1_('q0',_1) z_state_1_('q0',_4)
 z_state_2_('q0',_2) z_state_2_('q0',_0) z_state_2_('q0',_3)
 z_state_2_('q0',_1) z_state_2_('q0',_4) z_state_virtual_('q0',_2)
 z_state_virtual_('q0',_0) z_state_virtual_('q0',_3)
 z_state_virtual_('q0',_1) z_state_virtual_('q0',_4)
End
