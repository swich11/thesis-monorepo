import torch
import torchvision.transforms.functional as F


def crop(x: torch.Tensor, y: torch.Tensor) -> torch.Tensor:
    """
        Crops the input tensor x to be the same shape as y.
    """
    wd_x = (x.shape[2] - y.shape[2])
    wd_y = (x.shape[3] - y.shape[3])
    wd_xi = wd_x // 2
    wd_yi = wd_y // 2
    wd_xj = wd_xi + 1 if (wd_x % 2) else wd_xi
    wd_yj = wd_yi + 1 if (wd_y % 2) else wd_yi
    x = x[:, :, wd_xi:-wd_xj, wd_yi:-wd_yj]
    assert(x.shape == y.shape)
    return x


def calculate_depth_loss(x_res: torch.Tensor, x_real: torch.Tensor) -> torch.Tensor:
    x_real = crop(x_real, x_res)
    
    # sanatise the input to remove infinite depths
    # x_real = torch.where(torch.isinf(x_real), torch.full_like(x_real, -1.0, device=x_real.device), x_real)
    
    mask = ~torch.isinf(x_real)

    # apply gaussian smoothing on the result to ensure it doesn't overfit
    # we are not trying to get a 1 to 1 depth map here
    # just a very good, averaged approximation
    # x_res_blur = F.gaussian_blur(x_res, [5, 5])
    # d = torch.log(x_res[mask]) - torch.log(x_real[mask])
    # loss = torch.mean(d**2) - 0.5*torch.mean(d)**2
    loss = torch.mean(torch.abs(x_res[mask] - x_real[mask]))
    # loss = torch.mean((x_res_blur - x_real)**2)
    # loss = torch.mean(torch.linalg.vector_norm((x_res_blur - x_real), 2, dim=1))
    return loss



def calculate_flow_loss(x_res: torch.Tensor, x_real: torch.Tensor) -> torch.Tensor:
    x_real = crop(x_real, x_res)
    
    # specifically no blur for optical flow

    # summed vector loss, simple supervised distance loss
    # x_res_blur = F.gaussian_blur(x_res, [3, 3])
    mask = x_real != -1.0 # invalid values

    loss = torch.mean(torch.abs(x_res[mask] - x_real[mask]))
    # loss = torch.mean(torch.linalg.norm(x_res - x_real, 2, dim=1))
    return loss


def calculate_velocity_loss(x_res: torch.Tensor, x_real: torch.Tensor) -> torch.Tensor:
    # Vector norm for 6-DOF input
    return torch.linalg.vector_norm((x_res - x_real))