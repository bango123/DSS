import torch
<<<<<<< HEAD
from ..utils.mathHelper import dot, div, mul, det22, normalize, mm, inverse22, inverse33
from ..utils.matrixConstruction import convertWorldToCameraTransform, batchLookAt, batchAffineMatrix
#from pytorch_points.network import operations
from pytorch_points.utils.pc_utils import read_ply


class Camera:
    width = 256
    height = 256
    near = 0.1
    far = 10000000.0
    eps = 1e-9
    sv = width/2

    def __init__(self, device=None):
        self.device = device
        # position and rotation are the extrinsic camera parameters in the world coordinate system
        self.rotation = torch.eye(3, device=self.device)
        self.position = torch.tensor([0.0, 0.0, 0.0], device=self.device)


class PinholeCamera(Camera):
    focalLength = 15
    type = "pinhole"

    def __init__(self, device=None, focalLength=None, width=None, height=None, sv=None):
        Camera.__init__(self, device=device)
        self.principalPoint = torch.tensor([0.0, 0.0], dtype=torch.float).cuda()
        self.position = torch.tensor([0, 0, 20], dtype=torch.float).cuda()
        self.rotation = torch.eye(3, dtype=torch.float).cuda()
        self.rotation[2, 2] = -1
        if width is not None:
            self.width = width
        if height is not None:
            self.height = height
        if sv is not None:
            self.sv = sv
        if focalLength is not None:
            self.focalLength = focalLength

    def projectionMatrix(self):
        """
        Returns the 3x3 camera calibration matrix as torch tensor.
        Assume the camera screen size is 1x1, rescale it with width and height
        Flip the y angle sign here
        """
        if not hasattr(self, "_projMatrix"):
            self._projMatrix = torch.tensor([
                [self.focalLength*self.sv, 0.0, self.width/2],
                [0.0, -self.focalLength*self.sv, self.height/2],
                [.0, .0, 1]
            ]).cuda()
        return self._projMatrix

    def backProjectionMatrix(self):
        if not hasattr(self, "_invProjMatrix"):
            self._invProjMatrix = self.projectionMatrix().inverse()
        return self._invProjMatrix

    def world2CameraMatrix(self, rotation, position):
        """
        4x4 view matrix: P = K[R|t]
        """
        P = torch.eye(4, dtype=rotation.dtype).to(device=rotation.device)
        P = P.unsqueeze(0).expand(rotation.shape[0], -1, -1)
        (R, t) = convertWorldToCameraTransform(rotation, position)
        P[:, :3, :3] = R
        P[:, :3, -1] = t
        return P

    def projectPoints(self, cameraPoints):
        """
        Place points on camera plane in pixel coordinates
        """
        # projPoints = cameraPoints[:, 0:3]/cameraPoints[:, 2:3].detach()
        # pxlPoints = projPoints.matmul(self.projectionMatrix().transpose(0, 1))
        # projPoints = pinholeProject(cameraPoints[:, :3].contiguous(), self.width, self.height, self.focalLength)
        b, n, _ = cameraPoints.shape
        projPoints = torch.ones((b, n, 3), dtype=cameraPoints.dtype, device=cameraPoints.device)
        const = cameraPoints[:, :, 2].detach()
        projPoints[:, :, 0] = cameraPoints[:, :, 0]*self.focalLength*self.sv/const+self.width/2
        projPoints[:, :, 1] = -cameraPoints[:, :, 1]*self.focalLength*self.sv/const+self.height/2
        return projPoints

    def pixelDepth(self, inPlane):
        """
        Return the euclidean distance from the camera to the reprojected pixel on the normal plane
        """
        return inPlane[:, :, 2]

    def cameraAngles(self, cameraPoints, cameraNormals):
        """
        Set the point clouds' normalAngle: the dot product between the normal of a point and the viewing ray to the camera
        """
        # vector from points to camera
        camDir = -normalize(cameraPoints[:, 0:3], 1)
        normalAngle = dot(camDir, cameraNormals, 1)
        return normalAngle

    def backproject(self, projected, cameraPoints, cameraNormals):
        """
        projected: (2, B, N, 3) homogen points in pixelspace
        cameraPoints (B, N, 4): cloud points in camera coordinates
        cameraNormals (B, N, 3): cloud normals in camera coordinates
        """
        # Equation system to find y_0k with point p_k:
        # 1: (x0d - p_k) * n_k = 0 // orthogonal constraint
        # 2: p = x0d * t
        # merge: x0 * t * n_k - p_k * n_k = 0
        # solved for t:
        # t = (p_k * n_k) / (x3d_0k * n_k)
        # 1x3x3
        M = self.backProjectionMatrix().unsqueeze(0).unsqueeze(0)
        # 2xBxNx3
        x0d = projected.matmul(M.transpose(2, 3))
        # find t BxNx1
        normalAngle = torch.sum(cameraPoints[:, :, 0:3]*cameraNormals, dim=-1, keepdim=True)
        x0ts = normalAngle / torch.sum(x0d*cameraNormals[:, :, :3], dim=-1, keepdim=True)
        # find absolute(y0) in 3d camera space
        x0cam = x0d * x0ts
        return x0cam
=======
from pytorch3d.renderer.cameras import (PerspectiveCameras,
                                        look_at_view_transform)
>>>>>>> d96260c8c0b926ba2fd43d82eb3e0afd970a046a


class CameraSampler(object):
    """
    create camera transformations looking at the origin of the coordinate
    from varying distance

    Attributes:
        R, T: (num_cams_total, 3, 3) and (num_cams_total, 3)
        camera_type (Class): class to create a new camera
        camera_params (dict): camera parameters to call camera_type
            (besides R, T)
    """

    def __init__(self, num_cams_total, num_cams_batch,
                 distance_range=(5, 10), sort_distance=True,
                 return_cams=True,
                 camera_type=PerspectiveCameras, camera_params=None):
        """
        Args:
            num_cams_total (int): the total number of cameras to sample
            num_cams_batch (int): the number of cameras per iteration
            distance_range (tensor or list): (num_cams_total, 2) or (1, 2)
                the range of camera distance for uniform sampling
            sort_distance: sort the created camera transformations by the
                distance in ascending order
            return_cams (bool): whether to return camera instances or just the R,T
            camera_type (class): camera type from pytorch3d.renderer.cameras
            camera_params (dict): camera parameters besides R, T
        """
<<<<<<< HEAD
        if device is None:
            if points is not None:
                self.device = points.device
            else:
                self.device = torch.cuda.current_device()
        else:
            self.device = device
        self.closer = closer
        if filename is not None:
            self.allPositions = torch.from_numpy(read_ply(filename, nCam)).to(device=self.device)[:, :3]
            self.allPositions = self.allPositions.unsqueeze(0)
        else:
            # THIS IS NOT SUPPORTED IN THIS VERSION OF DSS BECAUSE IMPORTING
            # from pytorch_points.network import operations
            # BREAKS OTHER CODE I THAT IMPORTS THIS FILE (https://github.com/bango123/FluidReconstruction)
            raise NotImplementedError
            # sampleIdx, self.allPositions = operations.furthest_point_sample(points.cuda(), nCam, NCHW=False)
            # self.allPositions = self.allPositions.to(self.device)
            # if normals is not None:
            #     _, idx, _ = operations.faiss_knn(100, self.allPositions.cpu(), points.cpu(), NCHW=False)
            #     knn_normals = torch.gather(normals.unsqueeze(1).expand(-1, self.allPositions.shape[1], -1, -1), 2, idx.unsqueeze(-1).expand(-1, -1, -1, normals.shape[-1]))
            #     normals = torch.mean(knn_normals, dim=2).to(self.device)

        if points is not None:
            if points.dim() == 2:
                points = points.unsqueeze(0)
            maxP = torch.max(points, dim=1, keepdim=True)[0]
            minP = torch.min(points, dim=1, keepdim=True)[0]
            bb = maxP - minP
            offset = offset+bb
            if normals is not None:
                center = self.allPositions
                # self.allPositions = (torch.mean(normals, dim=1, keepdim=True))
                self.allPositions = normals+(torch.mean(normals, dim=1, keepdim=True))
                self.allPositions += torch.randn_like(self.allPositions) * 0.01
            else:
                center = torch.mean(points, dim=1, keepdim=True)
        else:
            center = torch.zeros([1, 1, 3], dtype=self.allPositions.dtype, device=self.allPositions.device)

        self.allPositions = self.allPositions*offset
        self.allPositions = center+self.allPositions
        # Bx1x3
        self.to = center.expand_as(self.allPositions)
        # BxNx3
        # self.ups = torch.tensor([0, 1, 0], dtype=self.to.dtype, device=self.to.device).view(1, 1, 3).expand_as(self.allPositions)
        # for sketchfab
        self.ups = torch.tensor([0, 0, 1], dtype=self.to.dtype, device=self.to.device).view(1, 1, 3).expand_as(self.allPositions)
        self.ups = self.ups + torch.randn_like(self.ups) * 0.0001
        self.rotation, self.position = batchLookAt(self.allPositions, self.to, self.ups)
        self.idx = 0
        self.length = self.rotation.shape[1]
        self.focalLength = focalLength
        self.camWidth = camWidth
        self.camHeight = camHeight
=======
        self.num_cams_batch = num_cams_batch
        self.num_cams_total = num_cams_total

        self.sort_distance = sort_distance
        self.camera_type = camera_type
        self.camera_params = {} if camera_params is None else camera_params

        # create camera locations
        distance_scale = distance_range[:, -1] - distance_range[:, 0]
        distances = torch.rand(num_cams_total) * distance_scale + \
            distance_range[:, 0]
        if sort_distance:
            distances, _ = distances.sort(descending=True)
        azim = torch.rand(num_cams_total) * 360 - 180
        elev = torch.rand(num_cams_total) * 180 - 90
        at = torch.rand((num_cams_total, 3)) * 0.1 - 0.05
        self.R, self.T = look_at_view_transform(
            distances, elev, azim, at=at, degrees=True)

        self._idx = 0
>>>>>>> d96260c8c0b926ba2fd43d82eb3e0afd970a046a

    def __len__(self):
        return (self.R.shape[0] + self.num_cams_batch - 1) // \
            self.num_cams_batch

    def __iter__(self):
        return self

    def __next__(self):
        if self._idx >= len(self):
            raise StopIteration
        start_idx = self._idx * self.num_cams_batch
        end_idx = min(start_idx + self.num_cams_batch, self.R.shape[0])
        cameras = self.camera_type(R=self.R[start_idx:end_idx],
                                   T=self.T[start_idx:end_idx],
                                   **self.camera_params)
        self._idx += 1
        return cameras
