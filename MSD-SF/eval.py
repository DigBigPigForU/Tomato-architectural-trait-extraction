import os
import cv2
import argparse
import numpy as np
from PIL import Image

import torch
import torch.nn as nn

from config import config
from utils.pyt_utils import ensure_dir, parse_devices
from engine.evaluator import Evaluator
from engine.logger import get_logger
from utils.metric import hist_info, compute_score
from dataloader.MSDSFDataset import MSDSFDataset
from models.builder import EncoderDecoder as segmodel
from dataloader.dataloader import ValPre

logger = get_logger()

class SegEvaluator(Evaluator):
    def func_per_iteration(self, data, device):
        rgb = data['rgb']
        ms = data['ms']
        d = data['d']
        label = data['label']
        name = data['fn']
        
        pred = self.sliding_eval_multimodal(rgb, ms, d, config.eval_crop_size, 
                                          config.eval_stride_rate, device)
        
        hist_tmp, labeled_tmp, correct_tmp = hist_info(config.num_classes, pred, label)
        results_dict = {
            'hist': hist_tmp, 
            'labeled': labeled_tmp, 
            'correct': correct_tmp
        }

        if self.save_path:
            ensure_dir(self.save_path)
            ensure_dir(f"{self.save_path}_color")

            fn = f"{name}.png"
            self.save_prediction(pred, fn, self.dataset.get_class_colors())

        return results_dict

    def sliding_eval_multimodal(self, rgb, ms, d, crop_size, stride_rate, device):
        return self.sliding_eval(rgb, crop_size, stride_rate, device)  

    def save_prediction(self, pred, filename, class_colors):
        color_img = Image.fromarray(pred.astype(np.uint8), mode='P')
        palette = np.array(class_colors).flatten().tolist()
        palette += [0] * (256 * 3 - len(palette))
        color_img.putpalette(palette)
        color_img.save(os.path.join(f"{self.save_path}_color", filename))
        
        cv2.imwrite(os.path.join(self.save_path, filename), pred)
        logger.info(f'Saved prediction to {os.path.join(self.save_path, filename)}')

    def compute_metric(self, results):
        hist = np.zeros((config.num_classes, config.num_classes))
        correct = labeled = 0
        
        for d in results:
            hist += d['hist']
            correct += d['correct']
            labeled += d['labeled']

        iou, miou, _, fIoU, mpa, pa = compute_score(hist, correct, labeled)
        return self.format_results(iou, fIoU, mpa, pa)

    def format_results(self, iou, freq_iou, mean_pa, pa):
        header = ["Class", "IoU", "Freq"]
        lines = [header]
        
        class_names = self.dataset.class_names
        for i, (iou_val, freq_val) in enumerate(zip(iou, freq_iou)):
            if i == 0 and config.ignore_background:
                continue
            lines.append([
                class_names[i], 
                f"{iou_val*100:.2f}%",
                f"{freq_val*100:.2f}%"
            ])
        
        footer = [
            ["Mean IoU", f"{miou*100:.2f}%"],
            ["Pixel Acc", f"{pa*100:.2f}%"],
            ["Mean PA", f"{mean_pa*100:.2f}%"]
        ]
        return self.print_table(lines + footer)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-e', '--epochs', default='last', type=str)
    parser.add_argument('-d', '--devices', default='0', type=str)
    parser.add_argument('-v', '--verbose', action='store_true')
    parser.add_argument('-p', '--save_path', default=None)
    args = parser.parse_args()

    devices = parse_devices(args.devices)
    network = segmodel(cfg=config, criterion=None, norm_layer=nn.BatchNorm2d)

    data_config = {
        'rgb_root': config.rgb_root,
        'ms_root': config.ms_root,
        'd_root': config.d_root,
        'label_root': config.label_root,
        'rgb_ext': config.rgb_ext,
        'ms_ext': config.ms_ext,
        'd_ext': config.d_ext,
        'label_ext': config.label_ext,
        'class_names': config.class_names,
        'class_colors': config.class_colors,
        'test_source': config.test_source
    }

    val_preprocessor = ValPre()
    dataset = MSDSFDataset(data_config, 'test', val_preprocessor)

    with torch.no_grad():
        evaluator = SegEvaluator(
            dataset=dataset,
            num_classes=config.num_classes,
            norm_mean=config.norm_mean,
            norm_std=config.norm_std,
            network=network,
            scales=config.eval_scale_array,
            flip=config.eval_flip,
            devices=devices,
            verbose=args.verbose,
            save_path=args.save_path
        )
        
        evaluator.run(
            ckpt_dir=config.checkpoint_dir,
            epochs=args.epochs,
            log_file=config.test_log_file,
            link_log=config.link_test_log
        )