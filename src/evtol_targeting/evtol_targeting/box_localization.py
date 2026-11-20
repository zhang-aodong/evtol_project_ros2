"""Instance boxes + actual projected LiDAR; no semantic masks or truth inputs."""
import numpy as np
from .clustering import VisibleTarget, _density_clusters


def normalize_detections(raw,options,width,height,view_count):
    """Map model class IDs explicitly; clip original-face pixel coordinates."""
    mapping={int(k):int(v) for k,v in options['class_map'].items()}
    result=[]
    for detection in raw:
        index=detection['view_index'];source=detection['source_class_id']
        confidence=float(detection['confidence']);box=np.asarray(detection['xyxy'],float)
        if type(index) is not int or not 0<=index<view_count or source not in mapping:continue
        if box.shape!=(4,) or not np.isfinite(box).all() or not np.isfinite(confidence):continue
        if not options['confidence']<=confidence<=1:continue
        box[[0,2]]=np.clip(box[[0,2]],0,width);box[[1,3]]=np.clip(box[[1,3]],0,height)
        if np.any(box[2:]-box[:2]<1):continue
        result.append(dict(view_index=index,source_class_id=int(source),class_id=mapping[source],
            confidence=confidence,xyxy=box.tolist()))
    return sorted(result,key=lambda b:-b['confidence'])[:view_count*options['max_det']]


def box_rays(boxes,k,width,height,budget=640,padding=4):
    """Distribute a strict per-view real-ray budget fairly across instances."""
    if not boxes:return []
    samples=[];allocations=np.full(len(boxes),budget//len(boxes),int)
    allocations[:budget%len(boxes)]+=1
    for box,count in zip(boxes,allocations):
        if count<1:continue
        x1,y1,x2,y2=np.asarray(box['xyxy'],float)
        center=np.array([(x1+x2)/2,(y1+y2)/2])
        # Include the centre and a dense interior grid before the motion rim.
        interior_count=max(1,int(count*.8));side=max(1,int(np.sqrt(interior_count)))
        xs=np.linspace(x1+.2*(x2-x1),x2-.2*(x2-x1),side)
        ys=np.linspace(y1+.2*(y2-y1),y2-.2*(y2-y1),side)
        grid=np.array(np.meshgrid(xs,ys)).reshape(2,-1).T
        remaining=max(0,count-1-len(grid));outer_side=max(1,int(np.ceil(np.sqrt(remaining))))
        xs=np.linspace(max(0,x1-padding),min(width-1,x2+padding),outer_side)
        ys=np.linspace(max(0,y1-padding),min(height-1,y2+padding),outer_side)
        rim=np.array(np.meshgrid(xs,ys)).reshape(2,-1).T
        xy=np.vstack((center,grid,rim))[:count]
        xy[:,0]=np.clip(xy[:,0],0,width-1);xy[:,1]=np.clip(xy[:,1],0,height-1)
        samples.append(xy)
    if not samples:return []
    xy=np.unique(np.concatenate(samples),axis=0)[:budget]
    return np.column_stack(((xy[:,0]-k[0,2])/k[0,0],(xy[:,1]-k[1,2])/k[1,1])).tolist()


def localize_boxes(world,projection,boxes,options,motion_pixels=None,ground_z=None):
    """Assign each ray to at most one box, cluster depth and reject background.

    The closest physically compact supported cluster near the box centre wins.
    A 2-D-only box never yields a position. Result positions are observed surface
    centres, not assumed vehicle centres or simulated actor origins.
    """
    stats=dict(boxes=len(boxes),boxes_without_depth=0,rejected_clusters=0,assigned_points=0)
    if not boxes:return [],stats
    ids=np.asarray(projection.point_indices,int);uv=np.asarray(projection.pixels_uv,float)
    depths=np.asarray(projection.depth_m,float)
    if not len(ids):stats['boxes_without_depth']=len(boxes);return [],stats
    margins=np.zeros(len(ids)) if motion_pixels is None else np.asarray(motion_pixels,float)
    if margins.shape!=(len(ids),) or not np.isfinite(margins).all() or np.any(margins<0):
        raise ValueError('Invalid per-ray motion margin')
    finite=np.isfinite(world[ids]).all(axis=1)&np.isfinite(depths)&(depths>0)
    # Points behind the first surface in the same camera pixel are occluded.
    # Projection pixels are integral and nonnegative. A scalar key preserves
    # identical-pixel occlusion while avoiding the costly Nx2 row sort.
    stride=int(uv[:,0].max())+1
    _,inverse=np.unique(uv[:,1].astype(np.int64)*stride+uv[:,0],return_inverse=True)
    nearest=np.full(inverse.max()+1,np.inf);np.minimum.at(nearest,inverse,depths)
    finite &= depths<=nearest[inverse]+options['cluster_radius_m']
    costs=np.full((len(ids),len(boxes)),np.inf)
    for j,b in enumerate(boxes):
        x1,y1,x2,y2=b['xyxy'];center=np.array([(x1+x2)/2,(y1+y2)/2])
        half=np.maximum(np.array([x2-x1,y2-y1])/2,1)
        inside=finite&(uv[:,0]>=x1-margins)&(uv[:,0]<=x2+margins)&(uv[:,1]>=y1-margins)&(uv[:,1]<=y2+margins)
        costs[inside,j]=np.sum(((uv[inside]-center)/half)**2,axis=1)+.01*(1-b['confidence'])
    owner=np.argmin(costs,axis=1);accepted=np.isfinite(costs[np.arange(len(ids)),owner])
    result=[]
    for j,b in enumerate(boxes):
        selected=np.flatnonzero(accepted&(owner==j));stats['assigned_points']+=len(selected)
        xyz=world[ids[selected]]
        labels=_density_clusters(xyz,options['cluster_radius_m'],options['min_lidar_points'])
        candidates=[]
        for label in np.unique(labels[labels>=0]):
            member=selected[labels==label];cluster=world[ids[member]]
            middle=np.median(cluster,axis=0);distance=np.linalg.norm(cluster-middle,axis=1)
            cutoff=max(options['cluster_radius_m']/2,np.median(distance)+3*1.4826*np.median(abs(distance-np.median(distance))))
            member=member[distance<=cutoff];cluster=world[ids[member]]
            if len(cluster)<options['min_lidar_points']:continue
            size=np.ptp(cluster,axis=0)
            if np.max(size)>options['max_drone_extent_m']:
                stats['rejected_clusters']+=1;continue
            if ground_z is not None and cluster[:,2].max()<=ground_z and size[2]<=.05:
                stats['rejected_clusters']+=1;continue
            # A corner background return is weak support for a box centred on
            # another object. The same finite motion allowance remains valid.
            center=np.mean(np.reshape(b['xyxy'],(2,2)),axis=0)
            half=np.maximum((np.array(b['xyxy'][2:])-b['xyxy'][:2])/2,1)
            centre_distance=np.min(np.linalg.norm((uv[member]-center)/(half+margins[member,None]),axis=1))
            if centre_distance>1.05:continue
            candidates.append((float(np.median(depths[member])),member,cluster))
        if not candidates:stats['boxes_without_depth']+=1;continue
        depth,member,cluster=min(candidates,key=lambda item:item[0])
        x1,y1,x2,y2=b['xyxy'];position=np.mean(cluster,axis=0)
        result.append(VisibleTarget(class_id=b['class_id'],position_body_m=tuple(position),
            distance_m=depth,lidar_point_count=len(cluster),mask_pixel_count=0,
            mean_confidence=b['confidence'],bbox_xywh=(int(x1),int(y1),int(x2-x1),int(y2-y1)),
            point_min_body_m=tuple(cluster.min(axis=0)),point_max_body_m=tuple(cluster.max(axis=0)),
            raw_point_count=len(member),component_count=1,point_indices=tuple(ids[member])))
    return result,stats
