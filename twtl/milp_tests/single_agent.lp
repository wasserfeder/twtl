\ LP format - for model browsing. Use MPS format to capture full model detail.
Minimize
  z_trans_0_0_('q0',_u'T0_init')_('q0',_u'T0_init')
   + 0 zeta_('q0',_u'T0_init')_('q0',_u'T0_init')
   + z_trans_0_0_('q0',_u'T0_init')_('q0',_u'accept_all')
   + 0 zeta_('q0',_u'T0_init')_('q0',_u'accept_all')
   + z_trans_0_0_('q0',_u'accept_all')_('v',_'v')
   + 0 zeta_('q0',_u'accept_all')_('v',_'v')
   + z_trans_0_1_('q0',_u'T0_init')_('q0',_u'T0_init')
   + 0 zeta_('q0',_u'T0_init')_('q0',_u'T0_init')
   + z_trans_0_1_('q0',_u'T0_init')_('q0',_u'accept_all')
   + 0 zeta_('q0',_u'T0_init')_('q0',_u'accept_all')
   + z_trans_0_1_('q0',_u'accept_all')_('v',_'v')
   + 0 zeta_('q0',_u'accept_all')_('v',_'v')
   + z_trans_0_2_('q0',_u'T0_init')_('q0',_u'T0_init')
   + 0 zeta_('q0',_u'T0_init')_('q0',_u'T0_init')
   + z_trans_0_2_('q0',_u'T0_init')_('q0',_u'accept_all')
   + 0 zeta_('q0',_u'T0_init')_('q0',_u'accept_all')
   + z_trans_0_2_('q0',_u'accept_all')_('v',_'v')
   + 0 zeta_('q0',_u'accept_all')_('v',_'v')
   + z_trans_1_1_('q0',_u'T0_init')_('q0',_u'T0_init')
   + 0 zeta_('q0',_u'T0_init')_('q0',_u'T0_init')
   + z_trans_1_1_('q0',_u'T0_init')_('q0',_u'accept_all')
   + 0 zeta_('q0',_u'T0_init')_('q0',_u'accept_all')
   + z_trans_1_1_('q0',_u'accept_all')_('v',_'v')
   + 0 zeta_('q0',_u'accept_all')_('v',_'v')
   + z_trans_2_2_('q0',_u'T0_init')_('q0',_u'T0_init')
   + 0 zeta_('q0',_u'T0_init')_('q0',_u'T0_init')
   + z_trans_2_2_('q0',_u'T0_init')_('q0',_u'accept_all')
   + 0 zeta_('q0',_u'T0_init')_('q0',_u'accept_all')
   + z_trans_2_2_('q0',_u'accept_all')_('v',_'v')
   + 0 zeta_('q0',_u'accept_all')_('v',_'v')
   + 0 z_state_virtual_('q0',_u'accept_all')
Subject To
 initialization: z_trans_virtual_0_('q0',_u'T0_init')_('q0',_u'T0_init')
   = 1
 flow: z_trans_0_0_('q0',_u'T0_init')_('q0',_u'accept_all')
   - z_trans_0_0_('q0',_u'accept_all')_('v',_'v')
   + z_trans_0_1_('q0',_u'T0_init')_('q0',_u'accept_all')
   - z_trans_0_1_('q0',_u'accept_all')_('v',_'v')
   + z_trans_0_2_('q0',_u'T0_init')_('q0',_u'accept_all')
   - z_trans_0_2_('q0',_u'accept_all')_('v',_'v')
   + z_trans_1_1_('q0',_u'T0_init')_('q0',_u'accept_all')
   - z_trans_1_1_('q0',_u'accept_all')_('v',_'v')
   + z_trans_2_2_('q0',_u'T0_init')_('q0',_u'accept_all')
   - z_trans_2_2_('q0',_u'accept_all')_('v',_'v')
   + z_trans_virtual_0_('q0',_u'T0_init')_('q0',_u'accept_all')
   - z_trans_virtual_0_('q0',_u'accept_all')_('v',_'v') = 0
 syncr: z_trans_0_0_('q0',_u'T0_init')_('q0',_u'T0_init')
   + z_trans_0_1_('q0',_u'T0_init')_('q0',_u'T0_init')
   + z_trans_0_2_('q0',_u'T0_init')_('q0',_u'T0_init')
   + z_trans_1_1_('q0',_u'T0_init')_('q0',_u'T0_init')
   + z_trans_2_2_('q0',_u'T0_init')_('q0',_u'T0_init')
   + z_trans_virtual_0_('q0',_u'T0_init')_('q0',_u'T0_init')
   - zeta_('q0',_u'T0_init')_('q0',_u'T0_init') <= 0
 syncr: zeta_('q0',_u'T0_init')_('q0',_u'accept_all') = 1
 syncr: z_trans_0_0_('q0',_u'T0_init')_('q0',_u'accept_all')
   + z_trans_0_1_('q0',_u'T0_init')_('q0',_u'accept_all')
   + z_trans_0_2_('q0',_u'T0_init')_('q0',_u'accept_all')
   + z_trans_1_1_('q0',_u'T0_init')_('q0',_u'accept_all')
   + z_trans_2_2_('q0',_u'T0_init')_('q0',_u'accept_all')
   + z_trans_virtual_0_('q0',_u'T0_init')_('q0',_u'accept_all')
   - zeta_('q0',_u'T0_init')_('q0',_u'accept_all') <= 0
 syncr: zeta_('q0',_u'T0_init')_('q0',_u'accept_all') = 1
 syncr: z_trans_0_0_('q0',_u'accept_all')_('v',_'v')
   + z_trans_0_1_('q0',_u'accept_all')_('v',_'v')
   + z_trans_0_2_('q0',_u'accept_all')_('v',_'v')
   + z_trans_1_1_('q0',_u'accept_all')_('v',_'v')
   + z_trans_2_2_('q0',_u'accept_all')_('v',_'v')
   + z_trans_virtual_0_('q0',_u'accept_all')_('v',_'v')
   - zeta_('q0',_u'accept_all')_('v',_'v') <= 0
 syncr: zeta_('q0',_u'accept_all')_('v',_'v') = 1
 state_transition: - z_trans_0_0_('q0',_u'T0_init')_('q0',_u'T0_init')
   - z_trans_0_0_('q0',_u'T0_init')_('q0',_u'accept_all')
   - z_trans_0_1_('q0',_u'T0_init')_('q0',_u'T0_init')
   - z_trans_0_1_('q0',_u'T0_init')_('q0',_u'accept_all')
   - z_trans_0_2_('q0',_u'T0_init')_('q0',_u'T0_init')
   - z_trans_0_2_('q0',_u'T0_init')_('q0',_u'accept_all')
   + z_state_0_('q0',_u'T0_init') = 0
 state_transition: - z_trans_0_0_('q0',_u'T0_init')_('q0',_u'accept_all')
   - z_trans_virtual_0_('q0',_u'T0_init')_('q0',_u'accept_all')
   + z_state_0_('q0',_u'accept_all') = 0
 state_transition: - z_trans_0_0_('q0',_u'accept_all')_('v',_'v')
   - z_trans_virtual_0_('q0',_u'accept_all')_('v',_'v')
   + z_state_0_('v',_'v') = 0
 state_transition: - z_trans_1_1_('q0',_u'T0_init')_('q0',_u'T0_init')
   - z_trans_1_1_('q0',_u'T0_init')_('q0',_u'accept_all')
   + z_state_1_('q0',_u'T0_init') = 0
 state_transition: - z_trans_0_1_('q0',_u'T0_init')_('q0',_u'accept_all')
   - z_trans_1_1_('q0',_u'T0_init')_('q0',_u'accept_all')
   + z_state_1_('q0',_u'accept_all') = 0
 state_transition: - z_trans_0_1_('q0',_u'accept_all')_('v',_'v')
   - z_trans_1_1_('q0',_u'accept_all')_('v',_'v') + z_state_1_('v',_'v')
   = 0
 state_transition: - z_trans_2_2_('q0',_u'T0_init')_('q0',_u'T0_init')
   - z_trans_2_2_('q0',_u'T0_init')_('q0',_u'accept_all')
   + z_state_2_('q0',_u'T0_init') = 0
 state_transition: - z_trans_0_2_('q0',_u'T0_init')_('q0',_u'accept_all')
   - z_trans_2_2_('q0',_u'T0_init')_('q0',_u'accept_all')
   + z_state_2_('q0',_u'accept_all') = 0
 state_transition: - z_trans_0_2_('q0',_u'accept_all')_('v',_'v')
   - z_trans_2_2_('q0',_u'accept_all')_('v',_'v') + z_state_2_('v',_'v')
   = 0
 state_transition:
   - z_trans_virtual_0_('q0',_u'T0_init')_('q0',_u'T0_init')
   - z_trans_virtual_0_('q0',_u'T0_init')_('q0',_u'accept_all')
   + z_state_virtual_('q0',_u'T0_init') = 0
 guard_sat_ts: - z_state_1_('q0',_u'accept_all')
   + zeta_('q0',_u'accept_all')_A <= 0
 guard_sat_product: zeta_('q0',_u'T0_init')_('q0',_u'accept_all')
   - zeta_('q0',_u'accept_all')_A <= 0
 guard_sat_ts: - z_state_1_('q0',_u'accept_all')
   + zeta_('q0',_u'accept_all')_A <= 0
 guard_sat_product: zeta_('q0',_u'T0_init')_('q0',_u'accept_all')
   - zeta_('q0',_u'accept_all')_A <= 0
 final: z_state_0_('v',_'v') + z_state_1_('v',_'v') + z_state_2_('v',_'v')
   + z_state_virtual_('v',_'v') >= 1
Bounds
Binaries
 zeta_('q0',_u'T0_init')_('q0',_u'T0_init')
 zeta_('q0',_u'T0_init')_('q0',_u'accept_all')
 zeta_('q0',_u'accept_all')_('v',_'v')
 zeta_('q0',_u'T0_init')_('q0',_u'T0_init')
 zeta_('q0',_u'T0_init')_('q0',_u'accept_all')
 zeta_('q0',_u'accept_all')_('v',_'v')
 zeta_('q0',_u'T0_init')_('q0',_u'T0_init')
 zeta_('q0',_u'T0_init')_('q0',_u'accept_all')
 zeta_('q0',_u'accept_all')_('v',_'v')
 zeta_('q0',_u'T0_init')_('q0',_u'T0_init')
 zeta_('q0',_u'T0_init')_('q0',_u'accept_all')
 zeta_('q0',_u'accept_all')_('v',_'v')
 zeta_('q0',_u'T0_init')_('q0',_u'T0_init')
 zeta_('q0',_u'T0_init')_('q0',_u'accept_all')
 zeta_('q0',_u'accept_all')_('v',_'v')
 zeta_('q0',_u'T0_init')_('q0',_u'T0_init')
 zeta_('q0',_u'T0_init')_('q0',_u'accept_all')
 zeta_('q0',_u'accept_all')_('v',_'v') zeta_('q0',_u'accept_all')_A
 zeta_('q0',_u'accept_all')_A
Generals
 z_trans_0_0_('q0',_u'T0_init')_('q0',_u'T0_init')
 z_trans_0_0_('q0',_u'T0_init')_('q0',_u'accept_all')
 z_trans_0_0_('q0',_u'accept_all')_('v',_'v')
 z_trans_0_1_('q0',_u'T0_init')_('q0',_u'T0_init')
 z_trans_0_1_('q0',_u'T0_init')_('q0',_u'accept_all')
 z_trans_0_1_('q0',_u'accept_all')_('v',_'v')
 z_trans_0_2_('q0',_u'T0_init')_('q0',_u'T0_init')
 z_trans_0_2_('q0',_u'T0_init')_('q0',_u'accept_all')
 z_trans_0_2_('q0',_u'accept_all')_('v',_'v')
 z_trans_1_1_('q0',_u'T0_init')_('q0',_u'T0_init')
 z_trans_1_1_('q0',_u'T0_init')_('q0',_u'accept_all')
 z_trans_1_1_('q0',_u'accept_all')_('v',_'v')
 z_trans_2_2_('q0',_u'T0_init')_('q0',_u'T0_init')
 z_trans_2_2_('q0',_u'T0_init')_('q0',_u'accept_all')
 z_trans_2_2_('q0',_u'accept_all')_('v',_'v')
 z_trans_virtual_0_('q0',_u'T0_init')_('q0',_u'T0_init')
 z_trans_virtual_0_('q0',_u'T0_init')_('q0',_u'accept_all')
 z_trans_virtual_0_('q0',_u'accept_all')_('v',_'v')
 z_state_0_('q0',_u'T0_init') z_state_0_('q0',_u'accept_all')
 z_state_0_('v',_'v') z_state_1_('q0',_u'T0_init')
 z_state_1_('q0',_u'accept_all') z_state_1_('v',_'v')
 z_state_2_('q0',_u'T0_init') z_state_2_('q0',_u'accept_all')
 z_state_2_('v',_'v') z_state_virtual_('q0',_u'T0_init')
 z_state_virtual_('q0',_u'accept_all') z_state_virtual_('v',_'v')
End
