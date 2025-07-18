\ LP format - for model browsing. Use MPS format to capture full model detail.
Minimize
  0 zeta_('q0',_u'T0_init')_('q0',_u'T0_init')
   + 0 zeta_('q0',_u'T0_init')_('q0',_u'accept_all')
   + 0 zeta_('q0',_u'accept_all')_('v',_'v')
   + 0 z_state_1_('q0',_u'T0_init')
   + 0 z_state_virtual_('q0',_u'accept_all') + 0 z_state_virtual_('v',_'v')
Subject To
 init_state: z_state_virtual_('q0',_u'T0_init') = 1
 flow: z_trans_0_1_('q0',_u'T0_init')_('q0',_u'accept_all')
   - z_trans_0_1_('q0',_u'accept_all')_('v',_'v')
   + z_trans_virtual_0_('q0',_u'T0_init')_('q0',_u'accept_all')
   - z_trans_virtual_0_('q0',_u'accept_all')_('v',_'v') = 0
 syncr: z_trans_0_1_('q0',_u'T0_init')_('q0',_u'T0_init')
   + z_trans_virtual_0_('q0',_u'T0_init')_('q0',_u'T0_init')
   - zeta_('q0',_u'T0_init')_('q0',_u'T0_init') <= 0
 syncr: z_trans_0_1_('q0',_u'T0_init')_('q0',_u'accept_all')
   + z_trans_virtual_0_('q0',_u'T0_init')_('q0',_u'accept_all')
   - zeta_('q0',_u'T0_init')_('q0',_u'accept_all') <= 0
 syncr: z_trans_0_1_('q0',_u'accept_all')_('v',_'v')
   + z_trans_virtual_0_('q0',_u'accept_all')_('v',_'v')
   - zeta_('q0',_u'accept_all')_('v',_'v') <= 0
 state_transition: - z_trans_0_1_('q0',_u'T0_init')_('q0',_u'T0_init')
   - z_trans_0_1_('q0',_u'T0_init')_('q0',_u'accept_all')
   + z_state_0_('q0',_u'T0_init') = 0
 state_transition:
   - z_trans_virtual_0_('q0',_u'T0_init')_('q0',_u'accept_all')
   + z_state_0_('q0',_u'accept_all') = 0
 state_transition: - z_trans_virtual_0_('q0',_u'accept_all')_('v',_'v')
   + z_state_0_('v',_'v') = 0
 state_transition: - z_trans_0_1_('q0',_u'T0_init')_('q0',_u'accept_all')
   + z_state_1_('q0',_u'accept_all') = 0
 state_transition: - z_trans_0_1_('q0',_u'accept_all')_('v',_'v')
   + z_state_1_('v',_'v') = 0
 state_transition:
   - z_trans_virtual_0_('q0',_u'T0_init')_('q0',_u'T0_init')
   - z_trans_virtual_0_('q0',_u'T0_init')_('q0',_u'accept_all')
   + z_state_virtual_('q0',_u'T0_init') = 0
 final: z_trans_0_1_('q0',_u'accept_all')_('v',_'v')
   + z_trans_virtual_0_('q0',_u'accept_all')_('v',_'v') >= 1
Bounds
Binaries
 zeta_('q0',_u'T0_init')_('q0',_u'T0_init')
 zeta_('q0',_u'T0_init')_('q0',_u'accept_all')
 zeta_('q0',_u'accept_all')_('v',_'v')
 zeta_('q0',_u'T0_init')_('q0',_u'T0_init')
 zeta_('q0',_u'T0_init')_('q0',_u'accept_all')
 zeta_('q0',_u'accept_all')_('v',_'v')
Generals
 z_trans_0_1_('q0',_u'T0_init')_('q0',_u'T0_init')
 z_trans_0_1_('q0',_u'T0_init')_('q0',_u'accept_all')
 z_trans_0_1_('q0',_u'accept_all')_('v',_'v')
 z_trans_virtual_0_('q0',_u'T0_init')_('q0',_u'T0_init')
 z_trans_virtual_0_('q0',_u'T0_init')_('q0',_u'accept_all')
 z_trans_virtual_0_('q0',_u'accept_all')_('v',_'v')
 z_state_0_('q0',_u'T0_init') z_state_0_('q0',_u'accept_all')
 z_state_0_('v',_'v') z_state_1_('q0',_u'T0_init')
 z_state_1_('q0',_u'accept_all') z_state_1_('v',_'v')
 z_state_virtual_('q0',_u'T0_init') z_state_virtual_('q0',_u'accept_all')
 z_state_virtual_('v',_'v')
End
