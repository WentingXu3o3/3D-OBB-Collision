# 3D-OBB-Collision
Collision Detection for 3D OBB of bounding box

```
def check_collision(obb1, obb2):
    # Get axes from the normalizedAxes of OBBs
    axes1 = obb1['normalizedAxes']
    axes2 = obb2['normalizedAxes']

    # Combine axes for both OBBs
    all_axes = np.concatenate((axes1, axes2))
    obb1['centroid'] = np.array(obb1['centroid'])
    obb2['centroid'] = np.array(obb2['centroid'])


    ###################################################
    ###### Check for separation along each axis #######
    ###################################################

    for axis in all_axes:
        # Project OBBs onto the axis: OBB在每个法线轴的投影
        projection1 = np.dot(np.dot(0.5,obb1['axesLengths']), np.abs(np.dot(obb1['normalizedAxes'], axis)))
        projection2 = np.dot(np.dot(0.5,obb2['axesLengths']), np.abs(np.dot(obb2['normalizedAxes'], axis)))
        
        # Find the distance between the centroids projected on the axis
        distance = abs(np.dot((obb2['centroid'] - obb1['centroid']), axis))
        
        # Check for overlap
        if not (distance <= (projection1 + projection2)):
            # Separating axis found, no collision
            return False
    
    # No separating axis found, collision occurred
    return True
'''
OBB ={
  'centroid' : [1*3],
  'axesLengths' : [1*3],
  'normalizedAxes' : [3*3]
}
'''
```
