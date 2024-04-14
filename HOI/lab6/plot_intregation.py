import matplotlib.pyplot as plt
import numpy as np

eta_array_rt = np.load('/home/tanakrit-ubuntu/IFROS/HOI/ee_plot_both.npy')
ee_array_rt = np.load('/home/tanakrit-ubuntu/IFROS/HOI/ee_plot.npy')

eta_array_tran = np.load('/home/tanakrit-ubuntu/IFROS/HOI/eta_plot_tran.npy')
ee_array_tran = np.load('/home/tanakrit-ubuntu/IFROS/HOI/ee_plot_tran.npy')

eta_array_b = np.load('/home/tanakrit-ubuntu/IFROS/HOI/eta_plot_both.npy')
ee_array_b = np.load('/home/tanakrit-ubuntu/IFROS/HOI/ee_plot_both.npy')


fig_joint = plt.figure()
ax = fig_joint.add_subplot(111, autoscale_on=False, xlim=(-2, 2), ylim=(-2, 2))
ax.set_title("Mobile Robot Plot")
ax.set_xlabel("X[m]")
ax.set_ylabel("Y[m]")
ax.grid()

plt.plot(eta_array_rt[:,0], eta_array_rt[:,1], label=" 'ROTATE' mobile base path")    
plt.plot(ee_array_rt[:,0], ee_array_rt[:,1], label=" 'ROTATE' end-effector path") 

plt.plot(eta_array_tran[:,0], eta_array_tran[:,1], label=" 'TRANSLATE' mobile base path")
plt.plot(ee_array_tran[:,0], ee_array_tran[:,1], label=" 'TRANSLATE' end-effector path")

plt.plot(eta_array_b[:,0], eta_array_b[:,1], label=" 'BOTH' mobile base path")
plt.plot(ee_array_b[:,0], ee_array_b[:,1], label=" 'BOTH' end-effector path")

ax.legend()

plt.show()  



