import os
import cv2
import torch
import numpy as np
import torch.utils.data as data

class MSDSFDataset(data.Dataset):
    def __init__(self, setting, split_name, preprocess=None, file_length=None):
        super(MSDSFDataset, self).__init__()
        self._split_name = split_name
        self._rgb_path = setting['rgb_root']
        self._rgb_format = setting['rgb_format']
        self._gt_path = setting['gt_root']
        self._gt_format = setting['gt_format']
        self._transform_gt = setting['transform_gt']
        self._depth_path = setting['depth_root']
        self._depth_format = setting['depth_format']
        self._ms_path = setting['ms_root']
        self._ms_format = setting['ms_format']
        self._train_source = setting['train_source']
        self._eval_source = setting['eval_source']
        self.class_names = setting['class_names']
        self._file_names = self._get_file_names(split_name)
        self._file_length = file_length
        self.preprocess = preprocess

    def __len__(self):
        if self._file_length is not None:
            return self._file_length
        return len(self._file_names)

    def __getitem__(self, index):
        if self._file_length is not None:
            item_name = self._construct_new_file_names(self._file_length)[index]
        else:
            item_name = self._file_names[index]
        
        # Load RGB image
        rgb_path = os.path.join(self._rgb_path, item_name + self._rgb_format)
        rgb = self._open_image(rgb_path, cv2.COLOR_BGR2RGB)

        # Load Ground Truth
        gt_path = os.path.join(self._gt_path, item_name + self._gt_format)
        gt = self._open_image(gt_path, cv2.IMREAD_GRAYSCALE, dtype=np.uint8)
        if self._transform_gt:
            gt = self._gt_transform(gt)

        # Load Depth image
        depth_path = os.path.join(self._depth_path, item_name + self._depth_format)
        depth = self._open_image(depth_path, cv2.IMREAD_GRAYSCALE)

        # Load Multispectral image
        ms_path = os.path.join(self._ms_path, item_name + self._ms_format)
        ms = self._open_image(ms_path, cv2.IMREAD_COLOR)

        # Preprocess the data if necessary
        if self.preprocess is not None:
            rgb, gt, depth, ms = self.preprocess(rgb, gt, depth, ms)

        # Convert to tensors if in training mode
        if self._split_name == 'train':
            rgb = torch.from_numpy(np.ascontiguousarray(rgb)).float()
            gt = torch.from_numpy(np.ascontiguousarray(gt)).long()
            depth = torch.from_numpy(np.ascontiguousarray(depth)).float()
            ms = torch.from_numpy(np.ascontiguousarray(ms)).float()

        # Prepare output dictionary
        output_dict = dict(
            data=rgb, 
            label=gt, 
            depth=depth, 
            ms=ms, 
            fn=str(item_name), 
            n=len(self._file_names)
        )

        return output_dict

    def _get_file_names(self, split_name):
        assert split_name in ['train', 'val']
        source = self._train_source
        if split_name == "val":
            source = self._eval_source

        file_names = []
        with open(source) as f:
            files = f.readlines()

        for item in files:
            file_name = item.strip()
            file_names.append(file_name)

        return file_names

    def _construct_new_file_names(self, length):
        assert isinstance(length, int)
        files_len = len(self._file_names)                          
        new_file_names = self._file_names * (length // files_len)   

        rand_indices = torch.randperm(files_len).tolist()
        new_indices = rand_indices[:length % files_len]

        new_file_names += [self._file_names[i] for i in new_indices]

        return new_file_names

    def get_length(self):
        return self.__len__()

    @staticmethod
    def _open_image(filepath, mode=cv2.IMREAD_COLOR, dtype=None):
        img = np.array(cv2.imread(filepath, mode), dtype=dtype)
        return img

    @staticmethod
    def _gt_transform(gt):
        return gt - 1 

    @classmethod
    def get_class_colors(*args):
        def uint82bin(n, count=8):
            """returns the binary of integer n, count refers to amount of bits"""
            return ''.join([str((n >> y) & 1) for y in range(count - 1, -1, -1)])

        N = 41
        cmap = np.zeros((N, 3), dtype=np.uint8)
        for i in range(N):
            r, g, b = 0, 0, 0
            id = i
            for j in range(7):
                str_id = uint82bin(id)
                r = r ^ (np.uint8(str_id[-1]) << (7 - j))
                g = g ^ (np.uint8(str_id[-2]) << (7 - j))
                b = b ^ (np.uint8(str_id[-3]) << (7 - j))
                id = id >> 3
            cmap[i, 0] = r
            cmap[i, 1] = g
            cmap[i, 2] = b
        class_colors = cmap.tolist()
        return class_colors