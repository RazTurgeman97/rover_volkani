import os
from PIL import Image
import matplotlib.pyplot as plt
import ament_index_python.packages

def combine_images(image_paths, out_path, title):
    images = [Image.open(p) for p in image_paths if os.path.exists(p)]
    if not images:
        print(f"No images found for {out_path}")
        return
    widths, heights = zip(*(img.size for img in images))
    total_width = sum(widths)
    max_height = max(heights)
    combined = Image.new('RGB', (total_width, max_height), (255,255,255))
    x_offset = 0
    for img in images:
        combined.paste(img, (x_offset, 0))
        x_offset += img.size[0]
    # Optionally add a title using matplotlib
    plt.figure(figsize=(total_width/100, (max_height+40)/100), dpi=100)
    import numpy as np
    plt.imshow(np.array(combined))
    plt.axis('off')
    plt.title(title)
    plt.tight_layout()
    plt.savefig(out_path, bbox_inches='tight')
    plt.close()

def main():
    share = ament_index_python.packages.get_package_share_directory('experiment_monitor')
    logs_root = os.path.join(share, 'logs')
    for exp_dir in sorted(os.listdir(logs_root)):
        exp_path = os.path.join(logs_root, exp_dir)
        if not os.path.isdir(exp_path):
            continue
        # Collect duration.png and error.png for attempts 1,2,3
        duration_imgs = []
        error_imgs = []
        for n in [1,2,3]:
            att_dir = os.path.join(exp_path, f'attempt_{n}')
            duration_imgs.append(os.path.join(att_dir, 'duration.png'))
            error_imgs.append(os.path.join(att_dir, 'error.png'))
        # Combine duration and error images side by side
        duration_out = os.path.join(exp_path, 'comparison_duration.png')
        error_out = os.path.join(exp_path, 'comparison_error.png')
        combine_images(duration_imgs, duration_out, f'{exp_dir} - Duration Comparison')
        combine_images(error_imgs, error_out, f'{exp_dir} - Error Comparison')
        # Optionally, stack duration and error comparison vertically
        if os.path.exists(duration_out) and os.path.exists(error_out):
            dur_img = Image.open(duration_out)
            err_img = Image.open(error_out)
            width = max(dur_img.width, err_img.width)
            total_height = dur_img.height + err_img.height
            final_img = Image.new('RGB', (width, total_height), (255,255,255))
            final_img.paste(dur_img, (0,0))
            final_img.paste(err_img, (0,dur_img.height))
            final_out = os.path.join(exp_path, 'comparison_all.png')
            final_img.save(final_out)
            print(f"Saved {final_out}")

if __name__ == '__main__':
    main()