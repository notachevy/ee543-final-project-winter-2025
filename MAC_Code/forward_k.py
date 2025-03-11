from MAC_Code.symbolic import *

def ForwardK_N(dh_table):
    t_final = np.matrix(np.eye(4))

    for joint in dh_table:
        # dh in alpha, a, d, theta
        # link in alpha, a, theta, d
        alpha, a, d, theta = joint
        t_final = t_final @ Link_N(alpha, a, theta, d)

    return t_final

def Forwardk_S(dh_table):

if __name__ == "__main__":
    t1, t2, t3, t4 = sp.symbols("t1 t2 t3 t4")
    # Link_S(alpha, a, theta, d)
    t_01 = Link_S(0,    0,      t1,  62.8)
    t_12 = Link_S(90,   0,      t2,  0)
    t_23 = Link_S(0,    101,    t3,  0)
    t_34 = Link_S(90,   0,      t4,  87.5)
    t_45 = Link_S(0,    0,      0,  125)

    t_05 = t_01 @ t_12 @ t_23 @ t_34 @ t_45

    print("Symbolic Representation")
    
    print("T_01")
    sp.pprint(t_01)
    print()

    print("T_12")
    sp.pprint(t_12)
    print()

    print("T_23")
    sp.pprint(t_23)
    print()

    print("T_34")
    sp.pprint(t_34)
    print()

    print("T_45")
    sp.pprint(t_45)
    print()
    
    print("T_05")
    sp.pprint(t_05)
    print()