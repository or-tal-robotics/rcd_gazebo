import numpy as np

# TO DO:
1) Get location of all objects from gazebo
2) Get location and velocity of robot
3) Estimate future path
4) Estimate targets probabilitys
5) Publish path 


Nt = 10
A = np.eye(l_size*Nt)
P = np.zeros((l_size*Nt,l_size*Nt))
H = np.zeros(Nt*l_size)
alpha = 0*np.ones((l_size,l_size*Nt))
z = np.zeros(l_size)
y = np.zeros(l_size)
Q = A
R = 0.01
for ii in range(0,Nsamples):
        if ii >= Nt:
                P = A.dot(P.dot(A.transpose())) + Q
                for jj in range(0,l_size):
                        z[jj] =  trends_data_np[ii,jj]
                        H[jj*Nt:(jj+1)*Nt] =  trends_data_np[(ii-Nt):(ii),jj]
                y = z - alpha.dot(H)
                S = R + H.dot((P.dot(H.transpose())))
                K = P.dot(H.transpose())/S
                alpha = alpha + np.outer(y,K)
                P = (np.eye(Nt*l_size)-np.outer(K,H)).dot(P)