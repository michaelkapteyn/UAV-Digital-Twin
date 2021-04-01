import numpy as np
import matplotlib.pyplot as plt
import json

class Planner:
    def __init__(self, gm):
        self.gm = gm
        self.states = self.gm.config["flat_states"]
        self.controls = self.gm.config["controls"]
        self.gamma = self.gm.config["gamma"]


    def planning_reward(self, state, cidx):
        alpha1 = 1.0
        alpha2 = 2.5
        r = alpha1*self.state_reward_function(state, self.controls[cidx]) + alpha2*self.gm.control_reward_function(cidx)
        return r


    def state_reward_function(self, state, control):
        # wrapper function for gm.health_reward_function - gets the ref_obs corresponding to a state and control, then evaluates the health_reward for that ref_obs.
        ref_observation_dict = self.gm.config["observations"][str(state[0])][str(state[1])][control]
        R = 0
        for idx, ref_obs in enumerate(ref_observation_dict.values()):
            if idx > 0 and idx <= self.gm.n_samples:
                r = self.gm.health_reward_function(ref_obs)
                prob = 1./self.gm.n_samples
                R += prob*r
        return R

    def transition_probabilities_for_state_and_control(self, state1, control):
        # returns a list T where T[j] is the probability from transitioning from state1 (first function input) to state j, under a given control (second function input)
        p1 = self.gm.config["transition_probabilities"][control]
        p2 = self.gm.config["transition_probabilities"][control]
        T = []
        for state2 in self.states:
            d1 = state2[0] - state1[0]
            d2 = state2[1] - state1[1]
            if state1[0] == 80 and state1[1] == 80 and state2[0] == 80 and state2[1] == 80:
                T.append(1.0)
            elif d1 == d2 == 0:
                T.append((1.-p1)*(1.-p2))
            elif d1 == 20 and d2 == 20:
                T.append(p1*p2)
            elif d1 == 20 and d2 == 0:
                T.append(p1*(1.-p2))
            elif d2 == 20 and d1 == 0:
                T.append(p2*(1.-p1))
            else:
                T.append(0.0)
        return T

    # Value iteration algorithm to compute a policy
    def getPolicy(self):
        maxit = 100
        Ns = len(self.states)
        Nc = len(self.controls)

        V = np.zeros((Ns,maxit))
        v = np.zeros((Nc,1))
        for it in range(2,100):
            for sIdx, s in enumerate(self.states):
                for cIdx, c in enumerate(self.controls):
                    v[cIdx] = self.planning_reward(s,cIdx) + np.dot(np.transpose(V[:,it-1]),self.transition_probabilities_for_state_and_control(s, c))
                V[sIdx,it]  = self.gamma*np.max(v);
        V =  V[:,-1];

        Q  =  np.zeros((Ns,Nc));
        for sIdx, s in enumerate(self.states):
            for cIdx, c in enumerate(self.controls):
                Q[sIdx,cIdx] = self.planning_reward(s,cIdx) + np.dot(V,self.transition_probabilities_for_state_and_control(s, c))

        bestQ = np.max(Q,1)
        self.bestU = np.argmax(Q,1);

        ## optional:
        # self.plotPolicy(self.bestU)

        # print('Utility values for each state:')
        # print(list(zip(self.states, self.bestU)))


        # def qmdppolicy(belief): # multiplies belief by utility
        #     print(self.bestU)
        #     print(np.dot(belief.flatten(),self.bestU))
        #     return int(round(np.dot(belief.flatten(),self.bestU)))

        def mdppolicy(state):
            stateidx = self.states.index(state)
            return self.bestU[stateidx]

        print("Found a policy!")
        for state in self.states:
            print("State {}".format(state) + " is assigned control {}".format(self.controls[mdppolicy(state)]))
        return mdppolicy

    # plot the policy using matplotlib
    def plotPolicy(self, U):
            fig, ax = plt.subplots()
            pol = ax.imshow(U.reshape((5,5)).T,origin='lower')
            labels = ['0', '20', '40','60','80']
            plt.xlabel('Component 1 state')
            plt.ylabel('Component 2 state')
            ax.set_xticks([0,1,2,3,4])
            ax.set_yticks([0,1,2,3,4])
            ax.set_xticklabels(labels)
            ax.set_yticklabels(labels)

            cbar = fig.colorbar(pol, ax=ax, ticks=[0, 1])
            plt.show()
