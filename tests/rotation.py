import numpy as np
import math

def quaternionMultiply(quaternion1, quaternion0):
    w0, x0, y0, z0 = quaternion0
    w1, x1, y1, z1 = quaternion1
    return np.array([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                     x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                     -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                     x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0], dtype=np.float64)
def changeOfBasis(basis,v):
    matrix = np.linalg.inv(basis)*np.array(v)
    return np.sum(matrix, axis=0)
def getRefCOM(hitPos, COM, orn):
    #change the quaternion from x,y,z,w to w,x,y,z to make more sense
    orn = np.array(orn)
    orn = np.concatenate((orn[3],orn[0:3]), axis = None)
    ornPrime = np.array([orn[0], -orn[1], -orn[2], -orn[3]])
    b1 = quaternionMultiply(quaternionMultiply(orn, [0,1,0,0]),ornPrime)
    b2 = quaternionMultiply(quaternionMultiply(orn, [0,0,1,0]),ornPrime)
    b3 = quaternionMultiply(quaternionMultiply(orn, [0,0,0,1]),ornPrime)
    b1 = np.delete(b1,0)
    b2 = np.delete(b2,0)
    b3 = np.delete(b3,0)
    v = -1.2*(hitPos-COM)
    if len(v) > 3:
        v = np.delete(v,0)
    print('new Basis, v')
    print(np.array([b1,b2,b3]), v)
    return changeOfBasis(np.array([b1,b2,b3]), v)
#vector
v = np.array([0,1,0,0])
hitPos = np.array([0,0, 0.5, 0.325])
hitOrn = np.array([-0.5,0.5,-0.5,0.5])
objectOrn = np.array([0,-0.707,0,0.707])
parentCOM= np.array([0,-0.0075,0.453, 0.312])
childCOM = np.array([0,0.0129, 0.49, 0.377])
v = hitPos - parentCOM

#quaternion
objectPrime = np.array([objectOrn[0], -objectOrn[1], -objectOrn[2], -objectOrn[3]])
hitPrime = np.array([hitOrn[0], -hitOrn[1], -hitOrn[2], -hitOrn[3]])

'''
print(v)
print(quaternion_multiply(quaternion_multiply(objectOrn,v), objectPrime))
print(quaternion_multiply(quaternion_multiply(hitOrn,hitPos - childCOM), hitPrime))
'''
#change of basis

#IMPORTANT: for pybullet quaternions go x,y,z,w not w,x,y,z like normal

p1 = quaternion_multiply(quaternion_multiply(objectOrn,[0,1,0,0]), objectPrime)
p2 = quaternion_multiply(quaternion_multiply(objectOrn,[0,0,1,0]), objectPrime)
p3 = quaternion_multiply(quaternion_multiply(objectOrn,[0,0,0,1]), objectPrime)
print("parents, then desired")
p1 = np.delete(p1,0)
p2 = np.delete(p2,0)
p3 = np.delete(p3,0)
v = np.delete(v,0)
print(p1,p2,p3)
print([0,0,-1], [0,-1,0], [-1,0,0])
print('input')
print(hitPos, parentCOM, objectOrn)

print("Parent: new Basis, v")
newBasis = np.array([p1,p2,p3])
v = 1.2*np.array([0.0073954,0.04701536,0.01561491])
print(newBasis, v)
print("Change of Basis results")
print(changeOfBasis(newBasis, v))



c1 = quaternion_multiply(quaternion_multiply(hitOrn,[0,1,0,0]), hitPrime)
c2 = quaternion_multiply(quaternion_multiply(hitOrn,[0,0,1,0]), hitPrime)
c3 = quaternion_multiply(quaternion_multiply(hitOrn,[0,0,0,1]), hitPrime)
print("children, then desired")
print(c1,c2,c3)
print([0,0,1],[-1,0,0], [0,-1,0])
c1 = np.delete(c1,0)
c2 = np.delete(c2,0)
c3 = np.delete(c3,0)
v2 = 1.2*(hitPos-childCOM)
v2 = np.delete(v2,0)

print("child: newBasis, v")
print(np.array([c1,c2,c3]), v2)
print("child result")
print(changeOfBasis(np.array([c1,c2,c3]), v2))
'''
x axis of parent is y of child
z axis of parent is -z axis of child
y axis of parent is x axis of child


Sum of these must add up
desired parentRefCOM = [-0.0125,-0.0375,0]
in child frame it is [-0.0375, 0.0125, 0]
desired childRefCOM = [0,0.05,0]
in parent frame it is [0.05,0,0]

'''
