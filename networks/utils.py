import torch
import torchvision.transforms.functional as IF
import torch.nn.functional as F


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
    assert(x.shape[2:] == y.shape[2:])
    return x


def calculate_depth_loss(x_res: torch.Tensor, x_real: torch.Tensor,
                         event1: torch.Tensor, event2: torch.Tensor) -> torch.Tensor:
    x_real = crop(x_real, x_res)
    mask = ~torch.isinf(x_real)

    residuals = x_real[mask] - x_res[mask]
    l_scale_invariant = torch.mean(residuals**2 - 1/(torch.sum(mask)**2) * (torch.sum(residuals)**2))
    



    return l_scale_invariant


def calculate_flow_loss(x_res: torch.Tensor, x_real: torch.Tensor,
                        event1: torch.Tensor, event2: torch.Tensor) -> torch.Tensor:
    x_real = crop(x_real, x_res)
    event1 = crop(event1, x_res)
    event2 = crop(event2, x_res)

    # blurring helps avoid sparsity error
    event_mask = ((event1 + event2).sum(dim=1).squeeze(dim=0) > 0.0)
    # event1 = IF.gaussian_blur(event1, [3, 3])
    # event2 = IF.gaussian_blur(event2, [3, 3])


    # photometric loss on smoothed events
    r = 0.5
    mu = 1e-3
    H = event1.shape[2]
    W = event1.shape[3]
    # get normalized grid
    y, x = torch.meshgrid(
        torch.linspace(-1, 1, H, device=event1.device),
        torch.linspace(-1, 1, W, device=event1.device),
        indexing='ij'
    )
    grid = torch.stack((x, y), dim=2)
    grid = grid.unsqueeze(0)
    normalized_flow = torch.zeros_like(x_res)
    # Add time to the flow
    normalized_flow[:, 0, :, :] = x_res[:, 0, :, :] / ((W - 1) / 2.0)
    normalized_flow[:, 1, :, :] = x_res[:, 1, :, :] / ((H - 1) / 2.0)
    grid = grid + normalized_flow.permute(0, 2, 3, 1)
    warped_event2 = F.grid_sample(
        event2,
        grid,
        mode='bilinear',
        padding_mode='border',
        align_corners=False
    )

    charbonnier = lambda x: (x**2 + mu**2)**r
    l_photo = charbonnier((event1 - warped_event2) * event_mask)
    l_photo = l_photo.sum() / (event_mask.sum() + 1e-6)


    dx = charbonnier(x_res[:, :, :, 1:] - x_res[:, :, :, :-1])
    dy = charbonnier(x_res[:, :, 1:, :] - x_res[:, :, :-1, :])
    l_smooth = (dx.mean() + dy.mean())

    mask = x_real != -1.0
    l_reg = 1e-4 * x_res[mask].abs().mean()
    # print(f"photometric: {l_photo}")
    # print(f"smooth: {l_smooth}")
    return l_photo + l_reg + 50*l_smooth


def calculate_AAE(x_res: torch.Tensor, x_real: torch.Tensor, event1: torch.Tensor, event2: torch.Tensor) -> torch.Tensor:
    event1 = crop(event1, x_res)
    event2 = crop(event2, x_res)
    x_real = crop(x_real, x_res)
    events = IF.gaussian_blur((event1 + event2), [5, 5]).sum(dim=1).unsqueeze(0)
    return torch.mean(torch.abs((x_real - x_res) * events))





def calculate_velocity_loss(x_res: torch.Tensor, x_real: torch.Tensor) -> torch.Tensor:
    # Vector norm for 6-DOF input
    return torch.linalg.vector_norm((x_res - x_real))