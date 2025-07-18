import gurobipy as grb
import networkx as nx
from gurobipy import *
from lomap import Fsa
from itertools import chain
from collections import defaultdict


class Multi_Rob_Milp():

    '''Create the MILP encoding for optimal relaxation of TL goals'''

    def __init__(self, n_agents, ts, product_dfa):

        self.n_agents = n_agents

        # Instantiate gurobi model; add decision vars
        self.create_model(ts,product_dfa)

        # Initialization constraints
        self.add_initialization_constr(ts,product_dfa)

        # Flow constraints
        self.add_flow_constr(ts,product_dfa)
        #
        # Synchronization constraints
        self.add_sync_constr(ts,product_dfa)
        #
        # State-transition constraints
        self.add_state_trans_constr(ts, product_dfa)
        #
        self.guard_satisfaction_constr(ts,product_dfa)

        # Constraint to reach the final state in DFA
        self.add_final_state_constr(ts,product_dfa)
        #
        #
        #
        self.optimize(ts, product_dfa)
        self.model.write('relaxation_original.lp')

        self.project_on_ts(ts,product_dfa)


        for v in self.model.getVars():
            print("vars:", v)


    def get_init_props(self, ts):
        return [ts.g.node[init_ts].get('prop', set()) for init_ts in ts.init]


    def get_init_transitions(self, dfa, init_props):

        next_state = lambda q, prop : [v for _, v, d in dfa.g.out_edges_iter(q, data=True)
                                                       if prop in d['symbols']]

        return [node for node in dfa.g.nodes() for prop in init_props if node in next_state(next(iter(dfa.init)), prop)]

    def create_model(self, ts, product_dfa):
        '''
        assume g is connected DAG product graph
        source is a node in g
        add virtual init node
        '''

        self.model = grb.Model()

        sinks = [final_state for final_state in product_dfa.final]
        print("goals:", sinks)
        print("product nodes: ", product_dfa.g.nodes())

        self.virtual_init = ('virtual')
        ts.g.add_node(self.virtual_init, attr_dict={'prop': set(['virtual'])})
        ts.init.add(self.virtual_init)
        ts.g.add_weighted_edges_from([(self.virtual_init, u, 0) for u in ts.init if u != self.virtual_init])


        print("product finals:", product_dfa.final)

        # # if len(sinks) > 1:
        # self.target = ('v', 'v')
        # product_dfa.g.add_node(self.target)
        # product_dfa.final.add(self.target)
        # product_dfa.g.add_edges_from([(u, self.target) for u in sinks if not u == self.target])

        # print("product finals:", product_dfa.final)


        # store the vars for easy retrieval

        ''' 
        Define variables and keep track of them
        self.z_trans_vars _ {u, v, q, q'} - integer variable; indicates # agents transitioning from state u to v in TS and state q to q' in the FSA
        self.zeta_vars_ {q, q'} - binary variable; indicates which edge in FSA is being counted towards the objective
        self.z_state_vars_ {u,q} - integer variable; represents # agents currently present at state u in TS and state q in FSA
        '''

        self.z_trans_vars = defaultdict(dict)
        self.zeta_vars = defaultdict(dict)
        self.z_state_vars = defaultdict(dict)

        for ts_edge in ts.g.edges():
            for dfa_edge in product_dfa.g.edges():

                z_name = 'z_trans_{}_{}_{}_{}'.format(ts_edge[0], ts_edge[1], dfa_edge[0], dfa_edge[1])
                z = self.model.addVar(vtype = grb.GRB.INTEGER, name = z_name)
                self.z_trans_vars[ts_edge[0],ts_edge[1]][dfa_edge[0],dfa_edge[1]] = z

                zeta_name = 'zeta_{}_{}'.format(dfa_edge[0], dfa_edge[1])
                zeta = self.model.addVar(vtype = grb.GRB.BINARY, name = zeta_name)
                self.zeta_vars[dfa_edge[0]][dfa_edge[1]] = zeta
        self.model.update()

        for ts_state in ts.g.nodes():
            for dfa_state in product_dfa.g.nodes():
                z_name = 'z_state_{}_{}'.format(ts_state, dfa_state)
                z = self.model.addVar(vtype = grb.GRB.INTEGER, name = z_name)
                self.z_state_vars[ts_state][dfa_state] = z
        self.model.update()
        print("zeta_trans:", self.z_trans_vars)

    def add_initialization_constr(self, ts, product_dfa):

        '''Initilization constraints work correctly because of the synchronization constraints'''

        # get set of APs true at deployment
        init_props = self.get_init_props(ts)
        print("init props:", init_props)

        # get initial transitions in product DFA based on the APs satisfied at deployment
        init_dfa_states = self.get_init_transitions(product_dfa, init_props)
        print("dfa init: ", init_dfa_states)

        # Initialization constraint
        z_init = grb.quicksum(self.z_trans_vars[u,v][next(iter(product_dfa.init)), dfa_state]
                              for u,v in ts.g.out_edges(self.virtual_init) for dfa_state in init_dfa_states)

        self.model.addConstr(z_init == self.n_agents, name = 'initialization')


        ## Alt: initialization defined on the state vars
        # z_state_init = self.z_state_vars[self.virtual_init][next(iter(product_dfa.init))]
        # self.model.addConstr(z_state_init == self.n_agents, name='init_state')

        self.model.update()

    def add_flow_constr(self, ts, product_dfa):

        # Get DFA states other than initial and final states set
        dfa_states = set(product_dfa.g.nodes()) - product_dfa.init - product_dfa.final
        print("dfa states: ", dfa_states)

        for q in dfa_states:
            ## Doubtful about this -- self loops being double counted

            pre_q = product_dfa.g.predecessors(q)
            succ_q = product_dfa.g.successors(q)
            print("pre, post:", pre_q, succ_q)

            in_flow = grb.quicksum(self.z_trans_vars[u,v][predecessor, q] for u,v in ts.g.edges() for predecessor in pre_q)
            out_flow = grb.quicksum(self.z_trans_vars[u,v][q, successor] for u,v in ts.g.edges() for successor in succ_q)

            self.model.addConstr(in_flow == out_flow, name='flow')


    def add_sync_constr(self, ts, product_dfa):

        '''Synchronize the automaton state across all agents'''

        for q, q_next in product_dfa.g.edges():
            z_sum = grb.quicksum(self.z_trans_vars[u,v][q,q_next] for u,v in ts.g.edges())
            self.model.addConstr(z_sum <= self.n_agents * self.zeta_vars[q][q_next], name = 'syncr')

        # for q in product_dfa.g.nodes():
            q_next = product_dfa.g.successors(q)
            zeta_sum = grb.quicksum(self.zeta_vars[q][q__] for q__ in q_next)
            self.model.addConstr(zeta_sum == 1, name = 'syncr')


    def add_state_trans_constr(self, ts, product_dfa):

        for ts_node in ts.g.nodes():
            ts_next = ts.g.successors(ts_node)
            ts_prev = ts.g.predecessors(ts_node)

            for dfa_node in product_dfa.g.nodes():

                if dfa_node not in product_dfa.final and len(ts_next):
                    q_next = product_dfa.g.successors(dfa_node)
                    n_transitions = grb.quicksum(self.z_trans_vars[ts_node, ts_nxt_state][dfa_node, dfa_next_state]
                                                 for ts_nxt_state in ts_next for dfa_next_state in q_next)

                elif dfa_node in product_dfa.final and len(ts_prev):
                    print("ts_prev:", ts_prev)
                    q_prev = product_dfa.g.predecessors(dfa_node)
                    n_transitions = grb.quicksum(self.z_trans_vars[ts_prev_state, ts_node][dfa_prev_state, dfa_node]
                                                 for ts_prev_state in ts_prev for dfa_prev_state in q_prev)

                else:
                    '''Node not reachable'''
                    continue
                self.model.addConstr(self.z_state_vars[ts_node][dfa_node] == n_transitions, name ='state_transition')

    def guard_satisfaction_constr(self, ts, product_dfa):

        '''The specification is assumed to be in DNF form - no negations even in front of APs'''

        # We need terms of the APs -- but this is a simpler case since the term is the AP
        self.zeta_term_dict = defaultdict(dict)
        for q, q_next,data in product_dfa.g.edges(data=True):
            zeta_count = 0
            # if q_next in product_dfa.final:
            #     continue
            print("guard", data)
            guard = data['symbols']

            for term in guard:
                if any(term):
                    print("term:", term)
                    zeta_term = 'zeta_{}_{}'.format(q_next, next(iter(term)))
                    zeta = self.model.addVar(vtype = grb.GRB.BINARY, name = zeta_term)
                    self.zeta_term_dict[q_next][next(iter(term))] = zeta

                    ts_nodes = [node for node in ts.g.nodes() if ts.g.node[node].get('prop', set()) == term]
                    z_state_sum = grb.quicksum(self.z_state_vars[x][q_next] for x in ts_nodes)
                    self.model.addConstr(zeta <= z_state_sum, name='guard_satisfaction')


                    zeta_count = grb.quicksum(self.zeta_term_dict[q_next][ap] for ap in term)
                    self.model.addConstr(self.zeta_vars[q][q_next] <= zeta_count, name = 'product state and term')


    def add_final_state_constr(self, ts, product_dfa):

        token_count = grb.quicksum(self.z_state_vars[ts_node][dfa_final]
                                   for ts_node in ts.g.nodes() for dfa_final in product_dfa.final)

        # token_count = grb.quicksum(self.z_state_vars[ts_node][self.target]
                                   # for ts_node in ts.g.nodes() )
        # token_count = grb.quicksum(self.z_trans_vars[u,v][q,product_dfa.final]
        #                            for u,v in ts.g.edges() for q in product_dfa.g.predecessors(next(iter(product_dfa.final))) )
        self.model.addConstr( token_count >= 1, name ='final')

    def optimize(self, ts, product_dfa):

        cost_travel =  grb.quicksum(d['weight'] * self.z_trans_vars[u,v][q,q_next]
                                    for u, v, d in ts.g.edges(data=True) for q, q_next in product_dfa.g.edges())
        # cost_travel =  grb.quicksum(1 * self.z_trans_vars[u,v][q,q_next]
        #                             for u, v in ts.g.edges() for q, q_next in product_dfa.g.edges())
        self.model.update()
        self.model.setObjective(cost_travel, grb.GRB.MINIMIZE)
        self.model.optimize()


    def project_on_ts(self, ts, product_dfa):

        # Extract the set of variables that were set to true
        true_vars = []

        for v in self.model.getVars():
            print("vars:", v)
            if v.getAttr(grb.GRB.Attr.X) > 0:
                if v.Varname not in true_vars:
                    true_vars.append(v.varName)
        print("true vars:", true_vars)

        # init dict for initial states with agent as key

        from_ts = self.virtual_init
        from_dfa = (next(iter(product_dfa.init)))
        overall_path = {}

        for each_agent in range(self.n_agents):
            reached_final = False
            path = []
            path.append(from_ts)

            while not reached_final:
                out_edges = list(ts.g.successors(from_ts))
                out_dfa = list(product_dfa.g.successors(from_dfa))
                for to_ts in out_edges:
                    for to_dfa in out_dfa:
                        #
                        # if str('z_state_{}_{}'.format(to_ts, to_dfa)) in true_vars:
                        #     path.append(to_ts)
                        #     from_ts = to_ts
                        #     from_dfa = to_dfa
                        #     # break

                        # if to_dfa == ('v', 'v') or to_dfa in product_dfa.final:
                        #     reached_final = True

                        if str('z_trans_{}_{}_{}_{}'.format(from_ts, to_ts, from_dfa, to_dfa)) in true_vars:
                            path.append(to_ts)

                        if to_dfa in product_dfa.final:
                            reached_final = True

                        from_ts = to_ts
                        from_dfa = to_dfa

            overall_path[each_agent] = path

        print("path: ", overall_path)