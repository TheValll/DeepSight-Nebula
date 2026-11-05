"""Known-radius sphere silhouette metrology for an operational TCP origin.

Tangent rays u obey ||C x u|| = radius. This avoids treating an off-axis sphere
silhouette as a fronto-parallel planar circle. No TCP orientation is calibrated.
"""
import cv2
import numpy as np
from scipy.optimize import least_squares


def fit_sphere_rays(pixel_points, k, distortion, radius_m, *, seed=0,
                    min_distance_m=0.1, max_distance_m=2.0, trials=700):
    pixels=np.asarray(pixel_points,dtype=np.float64).reshape(-1,2)
    if len(pixels)<25:raise ValueError('Too few sphere silhouette points')
    normalized=cv2.undistortPoints(pixels[:,None],k,distortion).reshape(-1,2)
    rays=np.c_[normalized,np.ones(len(normalized))]
    rays/=np.linalg.norm(rays,axis=1)[:,None]
    focal=float(np.sqrt(k[0,0]*k[1,1]))
    rng=np.random.default_rng(seed)
    best=None
    for _ in range(trials):
        ids=rng.choice(len(rays),3,replace=False)
        axis=np.cross(rays[ids[1]]-rays[ids[0]],rays[ids[2]]-rays[ids[0]])
        norm=np.linalg.norm(axis)
        if norm<1e-10:continue
        axis/=norm
        if axis[2]<0:axis=-axis
        cosine=float(axis@rays[ids[0]])
        sine2=1-cosine*cosine
        if not 1e-8<sine2<0.5:continue
        distance=radius_m/np.sqrt(sine2)
        if not min_distance_m<distance<max_distance_m:continue
        center=axis*distance
        residual=np.abs(np.linalg.norm(np.cross(rays,center),axis=1)-radius_m)*focal/distance
        inliers=residual<0.85
        score=int(inliers.sum())
        if best is None or score>best[0]:best=(score,center,inliers)
    if best is None or best[0]<25:raise ValueError('No reliable sphere tangent cone')
    center=best[1]
    for _ in range(3):
        distance=np.linalg.norm(center)
        error=np.abs(np.linalg.norm(np.cross(rays,center),axis=1)-radius_m)*focal/distance
        selected=error<0.85
        def residual(c):
            return (np.linalg.norm(np.cross(rays[selected],c),axis=1)-radius_m)*focal/np.linalg.norm(c)
        fit=least_squares(residual,center,loss='soft_l1',f_scale=0.3,max_nfev=100)
        center=fit.x
    distance=float(np.linalg.norm(center))
    residual=np.abs(np.linalg.norm(np.cross(rays,center),axis=1)-radius_m)*focal/distance
    selected=residual<0.85
    if center[2]<=0 or not min_distance_m<distance<max_distance_m:
        raise ValueError('Sphere center outside admissible range')
    axis=center/distance
    basis=np.cross(axis,[0.,1.,0.]);basis/=np.linalg.norm(basis)
    second=np.cross(axis,basis)
    angles=np.sort(np.mod(np.arctan2(rays[selected]@second,rays[selected]@basis),2*np.pi))
    gaps=np.diff(np.r_[angles,angles[0]+2*np.pi])
    coverage=float(np.degrees(2*np.pi-gaps.max()))
    quality={'rms_px':float(np.sqrt(np.mean(residual[selected]**2))),
             'inliers':int(selected.sum()),'points':len(pixels),
             'inlier_fraction':float(selected.mean()),'arc_degrees':coverage,
             'center_camera_m':center.tolist()}
    # Fingers and putty contribute many candidate edges; geometric support over
    # more than half the silhouette matters more than their raw outlier count.
    if quality['inliers']<80 or quality['inlier_fraction']<0.15 or coverage<170:
        raise ValueError(f'Insufficient visible sphere arc: {quality}')
    return center,quality,selected


def orange_sphere(image,k,distortion,marker_pose,marker_corners,diameter_m=0.04):
    """Extract visible orange-ball boundary near marker 8, excluding most clutter."""
    hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    corners=np.asarray(marker_corners).reshape(4,2)
    scale=float(np.mean(np.linalg.norm(corners-np.roll(corners,-1,axis=0),axis=1)))
    midpoint=corners.mean(axis=0)
    x0=max(0,int(midpoint[0]-4*scale));x1=min(image.shape[1],int(midpoint[0]+4*scale))
    y0=max(0,int(midpoint[1]-4*scale));y1=min(image.shape[0],int(midpoint[1]+4*scale))
    roi=hsv[y0:y1,x0:x1]
    mask=cv2.inRange(roi,np.array([5,105,90]),np.array([40,255,255]))
    contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
    contours=[c for c in contours if cv2.contourArea(c)>200]
    if not contours:raise ValueError('Orange calibration sphere not segmented')
    contour=max(contours,key=cv2.contourArea)
    boundary=np.zeros(mask.shape,np.uint8)
    cv2.drawContours(boundary,[contour],-1,255,1)
    near=cv2.dilate(boundary,np.ones((7,7),np.uint8))
    saturation=roi[:,:,1]
    edges=cv2.Canny(saturation,30,80)
    ys,xs=np.nonzero((edges>0)&(near>0))
    if len(xs)<30:raise ValueError('Sphere contour too weak')
    # Localize the edge along its gradient rather than using integer mask pixels.
    gx=cv2.Sobel(saturation,cv2.CV_32F,1,0,ksize=3)
    gy=cv2.Sobel(saturation,cv2.CV_32F,0,1,ksize=3)
    gradient=np.c_[gx[ys,xs],gy[ys,xs]]
    magnitude=np.linalg.norm(gradient,axis=1)
    selected=magnitude>20
    xy=np.c_[xs,ys].astype(np.float32)[selected]
    normal=gradient[selected]/magnitude[selected,None]
    offsets=np.linspace(-1.5,1.5,31,dtype=np.float32)
    sample=xy[:,None,:]+normal[:,None,:]*offsets[None,:,None]
    values=cv2.remap(saturation,sample[:,:,0],sample[:,:,1],cv2.INTER_CUBIC)
    peak=np.argmax(np.abs(np.gradient(values.astype(float),axis=1)),axis=1)
    refined=xy+normal*offsets[peak,None]
    pixels=refined+np.array([x0,y0])
    marker_distance=float(np.linalg.norm(marker_pose[:3,3]))
    center,quality,inliers=fit_sphere_rays(pixels,k,distortion,diameter_m/2,
        min_distance_m=max(0.1,marker_distance-0.25),max_distance_m=marker_distance+0.25)
    if np.linalg.norm(center-marker_pose[:3,3])>0.25:
        raise ValueError('Sphere too far from gripper marker')
    return center,quality,pixels,inliers
