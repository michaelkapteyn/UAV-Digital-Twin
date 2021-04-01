import numpy as np
import json
import re
from timeit import default_timer as timer
from copy import copy, deepcopy
from scipy.stats import norm
import operator
import matplotlib
from matplotlib import pyplot as plt

import pygraphviz
import pomegranate as pm
from pomegranate import BayesianNetwork
import tempfile

from digitaltwin.utils import *

class GraphicalModel(BayesianNetwork):
    def __init__(self, name):
        super().__init__(name)
        config_fpath='./src/digitaltwin/inputfiles/UAVconfig.json'
        self.config = read_json_file(config_fpath)
        self.config["flat_states"] = flatten_list(self.config["states"])

        self.config["ref_obs_lookup"] = {}
        for state1 in self.config["states"][0]:
            for state2 in self.config["states"][1]:
                sampled_obs = {}
                for key, val in self.config["observations"][str(state1)][str(state2)]["2g"].items():
                    if key != "mean":
                        sampled_obs[key] = [x/2.0 for x in val]
                self.config["ref_obs_lookup"][json.dumps([x/2.0 for x in self.config["observations"][str(state1)][str(state2)]["2g"]["mean"]])] = sampled_obs


        self.master_timestep = -1
        self.prediction_timestep = -1

        self.variables = {}
        self.variables["states"] = []
        self.variables["observations"] = []
        self.variables["ref_observations"] = []
        self.variables["controlPs"] = []
        self.variables["controlAs"] = []
        self.variables["rewards"] = []
        self.variables["initialcontrol"] = []
        # self.variables["e"] = []

        self.factors = {}
        self.factors["observation"] = []
        self.factors["ref_observation"] = []
        self.factors["transition"] = []
        self.factors["controlP"] = []
        self.factors["controlA"] = []
        self.factors["reward"] = []
        # self.factors["e"] = []

        self.n_samples = 30
        self.n_samples = np.max([self.n_samples,30]) # can't use more than 30 samples!
        # self.E_factor = self.get_e_factor()
        self.Q_factor = self.get_Q_factor()
        self.sigma = 125
        self.evidence = {}
        self.policy = None
        self.most_recent_control = 1 #assume start with 3g

    def compute_marginals(self):
        # use Pomegranate back-end (forward-backward algorithm) to compute joint probabilities in the PGM
        start = timer()
        self.bake()
        self.joint = self.compute_joint(self.evidence, 10)
        print("Time to compute joint at timestep {}: {}".format(self.master_timestep,timer()-start))

        start = timer()
        self.marginals = self.convert_joint_to_marginals(self.joint)
        print("Time to convert joint to marginals at timestep {}: {}".format(self.master_timestep,timer()-start))
        return

    def compute_joint(self, evidence, horizon):
        return self.predict_proba(evidence)

    def convert_joint_to_marginals(self,joint_list):
        ## Input: list of distribution objects or strings returned by pomegranate predict_proba method
        ## Output: dict: keys are state names, values are lists of lists - each list corresponds to a state dimension
        self.state_joints = {}
        state_dim = []
        state_names = [state.name for state in self.states]
        for dim in self.config["states"]:
            state_dim.append(len(dim))
        marginals = {}
        for state, state_name in zip(joint_list,state_names):
            marginals[state_name] = []

            if 'Ref.' in  state_name:
                if self.n_samples == 0:
                    marginals[state_name] = state.parameters[0]
                else:
                    ref_observation_dict = state.parameters[0]
                    sampled_obs_dict = {}
                    for key, val in ref_observation_dict.items():
                        for idx, cleanObservation in enumerate(self.config["ref_obs_lookup"][key].values()):
                            if idx < self.n_samples:
                                sampled_obs_dict[json.dumps(cleanObservation)] = val* 1./self.n_samples
                    marginals[state_name] = sampled_obs_dict

            elif 'Observation' in state_name:
                marginals[state_name] =  json.loads(state)
            elif 'ControlP' in state_name:
                marginals[state_name] = [[state.parameters[0][control] for control in self.config["controls"] ]] # list of lists for consistency with state...allows multivariate control
            elif 'ControlA' in state_name:
                marginals[state_name] = state
            elif 'Damage' in state_name:
                posterior_mat = np.zeros(tuple(state_dim))

                for key,val in state.parameters[0].items():
                    keytuple = list(eval(key))
                    for kidx, k in enumerate(keytuple):
                        keytuple[kidx]=self.config["states"][kidx].index(k)
                    posterior_mat[tuple(keytuple)] = val
                for dim in range(len(state_dim)):
                    marginals[state_name].append(list(np.sum(posterior_mat,1-dim)))

                self.state_joints[state_name] = posterior_mat
        return marginals








    """
    Node callbacks
    """
    def process_new_observation(self, sensor_measurement):
        self.add_new_obs_node(self.master_timestep,list(sensor_measurement))


    def process_new_state(self):
        self.master_timestep += 1
        self.prediction_timestep  = max(self.prediction_timestep, self.master_timestep)
        self.add_new_state_node(self.master_timestep)
        self.add_new_ref_obs_node(self.master_timestep)
        self.add_new_controlP_node(self.master_timestep)

    def process_new_control(self, control):
        self.add_new_controlA_node(self.master_timestep, control)

        # add control to marginals dictionary to avoid having to resolve...
        self.marginals["ControlA {}".format(self.master_timestep)] = [[]]
        for idx,state in enumerate(self.config["controls"]):
            if control[0] == idx:
                self.marginals["ControlA {}".format(self.master_timestep)][0].append(1.0)
            else:
                self.marginals["ControlA {}".format(self.master_timestep)][0].append(0.0)
        self.most_recent_control = control[0]

    def prepare_prediction(self, t_predict):
        # create nodes and edges for prediction timesteps
        for t in range(self.master_timestep+1, t_predict+1):
            self.add_new_state_node(t)
            self.add_new_controlP_node(t)
            self.add_new_ref_obs_node(t)
        self.prediction_timestep = t_predict














    """
    Functions to add nodes to graph
    """
    def add_new_state_node(self, t):
        if len(self.variables["states"]) > t:
            # the state already exists in the graph.
            return
        else:
            if t == 0:
                P = self.get_prior_factor()
                self.factors["transition"].append(pm.DiscreteDistribution(P)) # add prior factor
                self.variables["states"].append(pm.State( self.factors["transition"][t], name="Damage {}".format(t)))
                self.add_node(self.variables["states"][t])
            else:
                T = self.get_transition_factor()
                # check if we have a controlA
                if len(self.variables["controlAs"]) > t-1:
                    self.factors["transition"].append(pm.ConditionalProbabilityTable(T, [self.factors["transition"][t-1],self.factors["controlA"][t-1]]))
                else:
                    self.factors["transition"].append(pm.ConditionalProbabilityTable(T, [self.factors["transition"][t-1],self.factors["controlP"][t-1]]))

                # add RV as a node in the graph
                self.variables["states"].append(pm.State( self.factors["transition"][t], name="Damage {}".format(t)))
                self.add_node(self.variables["states"][t])

                # connect node via transition edge
                self.add_edge(self.variables["states"][t-1], self.variables["states"][t] )

                # connect node via control edge
                if len(self.variables["controlAs"]) > t-1:
                    self.add_edge(self.variables["controlAs"][t-1], self.variables["states"][t] )
                else:
                    self.add_edge(self.variables["controlPs"][t-1], self.variables["states"][t] )

    def add_new_obs_node(self, t, sensor_measurement):
        O = self.get_observation_factor(sensor_measurement)
        self.factors["observation"].append(pm.ConditionalProbabilityTable(O, [self.factors["transition"][t]]))
        self.variables["observations"].append(pm.State( self.factors["observation"][t], name="Observation {}".format(t)))
        self.evidence["Observation {}".format(t)] = json.dumps(sensor_measurement)
        # add RV as a node in the graph
        self.add_node(self.variables["observations"][t])
        # connect node via observation edge
        self.add_edge(self.variables["states"][t], self.variables["observations"][t] )

    def add_new_ref_obs_node(self, t):
        if len(self.variables["ref_observations"]) > t:
            #node is already in the graph
            return
        else:
            self.factors["ref_observation"].append(pm.ConditionalProbabilityTable(self.Q_factor, [self.factors["transition"][t]]))
            self.variables["ref_observations"].append(pm.State(self.factors["ref_observation"][t], name="Ref. Observation {}".format(t)))

            # add RV as a node in the graph
            self.add_node(self.variables["ref_observations"][t])
            # connect node via ref observation edge
            self.add_edge(self.variables["states"][t], self.variables["ref_observations"][t] )

    def add_new_controlP_node(self,t):
        # 'Predicted' or estimated control node
        if len(self.variables["controlPs"]) > t:
            #node is already in the graph
            return
        else:
            C = self.get_controlP_factor()
            self.factors["controlP"].append(pm.ConditionalProbabilityTable(C, [self.factors["transition"][t]]))
            self.variables["controlPs"].append(pm.State(self.factors["controlP"][t], name="ControlP {}".format(t)))

            # add RV as a node in the graph
            self.add_node(self.variables["controlPs"][t])
            # connect node via observation edge
            self.add_edge(self.variables["states"][t], self.variables["controlPs"][t] )

    def add_new_controlA_node(self, t, control):
        # 'Actual' or enacted control node
        C = self.get_controlA_factor(control)
        self.factors["controlA"].append(pm.DiscreteDistribution(C))
        self.variables["controlAs"].append(pm.State(self.factors["controlA"][t], name="ControlA {}".format(t)))

        # add RV as a node in the graph
        self.add_node(self.variables["controlAs"][t])
        if len(self.variables["states"]) > t+1:
            T = self.get_transition_factor()
            self.factors["transition"][t+1].__init__ (T, [self.factors["transition"][t],self.factors["controlA"][t]])
            self.variables["states"][t+1].__init__(self.factors["transition"][t+1], name="Damage {}".format(t+1))
            for idx, (a,b) in enumerate(self.edges):
                if a.name == "ControlP {}".format(t) and b.name == "Damage {}".format(t+1):
                    self.edges[idx] = (self.variables["controlAs"][t], self.variables["states"][t+1])











    """
    Functions to get factors, i.e. conditional probability tables encoded by edges in the graph
    """
    def get_transition_factor(self): # p(D_t | D_t-1, U_t-1)
        T = []
        for state1 in self.config["flat_states"]:
            for state2 in self.config["flat_states"]:
                d1 = state2[0] - state1[0]
                d2 = state2[1] - state1[1]
                for control in self.config["controls"]:
                    p1 = self.config["transition_probabilities"][control]
                    p2 = self.config["transition_probabilities"][control]

                    if state1[0] == 80 and state1[1] == 80 and state2[0] == 80 and state2[1] == 80:
                        T.append([str(state1),control,str(state2),1.0])
                    elif d1 == d2 == 0:
                        T.append([str(state1),control,str(state2),(1.-p1)*(1.-p2)])
                    elif d1 == 20 and d2 == 20:
                        T.append([str(state1),control,str(state2),p1*p2])
                    elif d1 == 20 and d2 == 0:
                        T.append([str(state1),control,str(state2),p1*(1.-p2)])
                    elif d2 == 20 and d1 == 0:
                        T.append([str(state1),control,str(state2),p2*(1.-p1)])
                    else:
                        T.append([str(state1),control,str(state2),0.0])
        return T

    def get_observation_factor(self, m): # p(O_t | D_t)
        prob = np.zeros((len(self.config["flat_states"]),1))
        for idx,state in enumerate(self.config["flat_states"]):
            if self.most_recent_control == 0:
                cleanObservation = [x/2.0 for x in self.config["observations"][str(state[0])][str(state[1])]["2g"]["mean"]]
                scalefactor = 2.0
            elif self.most_recent_control == 1:
                cleanObservation = [x/3.0 for x in self.config["observations"][str(state[0])][str(state[1])]["3g"]["mean"]]
                scalefactor = 3.0
            for sensIdx in range(len(cleanObservation)):
                prob[idx] += np.log(norm.pdf(m[sensIdx], cleanObservation[sensIdx], self.sigma/np.sqrt(scalefactor)))
            prob[idx] = np.exp(prob[idx])
        prob = prob/np.linalg.norm(prob,1)
        eps = np.finfo(float).eps
        prob[prob < eps] = 0
        O = []
        for idx, state in enumerate(self.config["flat_states"]):
            O.append([str(state), json.dumps(m), float(prob[idx])])
        return O

    def get_Q_factor(self): # p(Q_t | D_t)
        Q = []
        # normalized measurements
        for idx, state in enumerate(self.config["flat_states"]):
            ref_obs = self.config["observations"][str(state[0])][str(state[1])]["2g"]["mean"]
            cleanObservation = [x/2.0 for x in ref_obs]
            for other_state in self.config["flat_states"]:
                if (state is other_state):
                    p = 1.0
                else:
                    p = 0.0
                Q.append([str(other_state), json.dumps(cleanObservation), p])
        return Q

    def get_controlP_factor(self): # p(U_t | D_t)
        if self.policy is None: # resort to a default policy
            C = []
            for idx, state in enumerate(self.config["flat_states"]):
                if state[0] >= 20 and state[1] >= 20:
                    C.append([str(state), '2g', 1.0])
                    C.append([str(state), '3g', 0.0])
                else:
                    C.append([str(state), '2g', 0.0])
                    C.append([str(state), '3g', 1.0])
        else:
            C = []
            for idx, state in enumerate(self.config["flat_states"]):
                cidx = self.policy(state)
                C.append([str(state), self.config["controls"][cidx], 1.0])
                C.append([str(state), self.config["controls"][1-cidx], 0.0])
        return C



    def get_controlA_factor(self,control): # This is a deterministic node that encodes an enacted U_t
        C = {}
        for idx,state in enumerate(self.config["controls"]):
            if idx == control[0]:
                C[state] = 1.0
            else:
                C[state] = 0.0
        return C


    def get_prior_factor(self): # p(D_0)
        P = {}
        for idx,state in enumerate(self.config["flat_states"]):
            if idx == 0:
                P[str(state)] = 1.0
            else:
                P[str(state)] = 0.0
        return P














    """
    Reward Functions
    """
    def health_reward_function(self, ref_obs):
        max_strain = 2500
        return 1.0 - (np.max(ref_obs) / max_strain)

    def control_reward_function(self, control):
        return 0.2*control - 0.1 #control == 0 corresponds to 2g

    def observation_reward_function(self, observation, ref_observation):
        # r = -1.0 * np.mean(np.linalg.norm(np.array(observation)-np.array(ref_observation),2)) # MSE)
        r = -1.0/125*np.mean( np.abs(np.array(observation)-np.array(ref_observation) )) # MAE discrepancy

        return r

    def evaluate_reward(self, t): # evaluates a reward function R(D_t,Q_t,U_t,O_t)
        def evaluate_state_reward(t):
            state_names = [state.name for state in self.states]
            R_state = 0
            state = [s for s,name in zip(self.joint,state_names) if 'Damage {}'.format(t) in name]
            for key,prob in state[0].parameters[0].items():
                keytuple = list(eval(key))
                state1 = keytuple[0]
                state2 = keytuple[1]
                r = self.state_reward_function(state1, state2)
                R_state += prob*r
                R_state_var += prob*np.power(r,2) #E[X]^2
            R_state_var = R_state_var - np.power(R_state,2) #var = E[x^2] - E[X]^2
            return R_state, R_state_var

        def evaluate_health_reward(t):
            ref_observation_dict = self.marginals["Ref. Observation {}".format(t)]
            R_health = 0.0
            R_health_var = 0.0
            for ref_obs, prob in ref_observation_dict.items():
                ref_obs = json.loads(ref_obs)
                r = self.health_reward_function(ref_obs)
                R_health += prob*r
                R_health_var += prob*np.power(r,2) #E[X]^2
            R_health_var = R_health_var - np.power(R_health,2) #var = E[x^2] - E[X]^2
            return R_health, R_health_var

        def evaluate_control_reward(t):
            if t < self.master_timestep:
                control_dict = self.marginals["ControlA {}".format(t)].parameters[0]
            else:
                control_dict = {}
                for control, prob in zip( self.config["controls"] , self.marginals["ControlP {}".format(t)][0]):
                    control_dict[control] = prob
            R_control = 0.0
            R_control_var = 0.0
            for c, prob in control_dict.items():
                cidx = self.config["controls"].index(c)
                r = self.control_reward_function(cidx)
                R_control += prob*r
                R_control_var += prob*np.power(r,2) #E[X]^2
            R_control_var = R_control_var - np.power(R_control,2) #var = E[x^2] - E[X]^2
            return R_control, R_control_var

        def evaluate_outputerror_reward(t):
            observation = self.marginals["Observation {}".format(t)]
            ref_observation_dict = self.marginals["Ref. Observation {}".format(t)]
            R_error = 0.0
            R_error_var = 0.0
            for ref_observation, prob in ref_observation_dict.items():
                ref_observation = json.loads(ref_observation)
                r = self.observation_reward_function(observation, ref_observation)
                R_error += prob*r
                R_error_var += prob*np.power(r,2) #E[X]^2
            R_error_var = R_error_var - np.power(R_error,2) #var = E[x^2] - E[X]^2
            return R_error, R_error_var

        def evaluate_policy_reward(t):
            controlA = self.marginals["ControlA {}".format(t)][0]
            controlP = self.marginals["ControlP {}".format(t)][0]
            cidxA = float(np.dot(np.arange(len(controlA)),np.array(controlA)))
            cidxP = float(np.dot(np.arange(len(controlP)),np.array(controlP)))
            p = np.abs(cidxA - cidxP)
            return p


        R_health, R_health_var = evaluate_health_reward(t)
        R_control, R_control_var = evaluate_control_reward(t)

        if t < self.master_timestep:
            R_error, R_error_var = evaluate_outputerror_reward(t)
        else:
            R_error = 0.0
            R_error_var = 0.0

        alpha1 = 1.
        alpha2 = 1. # try 0.005, 0.02, or 0.07
        alpha3 = 1.
        alpha4 = 1./20

        R = alpha1*R_health + alpha2*R_control + alpha4*R_error
        R_var = alpha1*R_health_var + alpha2*R_control_var + alpha4*R_error_var

        return R, R_var, R_health, R_health_var, R_control, R_control_var, R_error, R_error_var









    """
    Plotting utils
    """
    def to_string(self):
        G = pygraphviz.AGraph(directed=True,rankdir="LR")
        G.layout(prog='dot')
        for state in self.states:
            t = int(re.findall(r'\d+', state.name)[0])
            if t <=self.master_timestep:
                c = 'blue'
            else:
                c = 'red'
            if "Damage" in state.name:
                G.add_node(state.name, color=c, group = 'states', rank='min')
            else:
                G.add_node(state.name, color=c)

        for parent, child in self.edges:
            G.add_edge(parent.name, child.name)

        for t in range(self.prediction_timestep+1):
            if t <=self.master_timestep:
                c = 'blue'
            else:
                c = 'red'
            G.add_node("Reward {}".format(t), color=c)
            G.add_edge("Damage {}".format(t),"Reward {}".format(t))
            G.add_edge("Ref. Observation {}".format(t),"Reward {}".format(t))
            G.add_edge("ControlP {}".format(t),"Reward {}".format(t))
        for t in range(self.master_timestep+1):
            G.add_edge("Observation {}".format(t),"Reward {}".format(t))
            G.add_edge("ControlA {}".format(t),"Reward {}".format(t))

        # add dashed edges between controlP and controlA
        for state in self.states:
            if "ControlA" in state.name:
                G.add_edge(state.name.replace("A", "P"),state.name,style='dashed')
        return G.string()

        def remove_node(self,n):
            for i, state in enumerate(self.states):
                if state.name == n.name:
                    del self.states[i]
                    return

        def remove_edge(self,a,b):
            for i, (child, parent) in enumerate(self.edges):
                if (child.name == a.name) and (parent.name == b.name):
                    del self.edges[i]
                    return

        def print_nodes(self):
            print("\n")
            print("Printing Graph Nodes:")
            for s in self.states:
                print(s.name)
            print("\n")

        def print_edges(self):
            print("\n")
            print("Printing Graph Edges:")
            for a,b in self.edges:
                print("{} -> {}".format(a.name,b.name))
            print("\n")
