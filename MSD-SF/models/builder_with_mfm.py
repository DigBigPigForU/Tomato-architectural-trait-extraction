import torch
import torch.nn as nn
import torch.nn.functional as F

from utils.init_func import init_weight
from utils.load_utils import load_pretrain
from functools import partial

from engine.logger import get_logger

logger = get_logger()

class EncoderDecoder(nn.Module):
    def __init__(self, cfg=None, criterion=nn.CrossEntropyLoss(reduction='mean', ignore_index=255), norm_layer=nn.BatchNorm2d):
        super(EncoderDecoder, self).__init__()
        self.channels = [64, 128, 320, 512]
        self.norm_layer = norm_layer
        
        # ����MFMģ���ʼ��
        self.mfm = MultiBandFusionModule(rgb_channels=3, ms_channels=cfg.ms_channels)  # �����û�ȡ�����ͨ����
        
        # ��ʼ��Backbone������ԭ���߼���
        if cfg.backbone == 'mit_b5':
            logger.info('Using backbone: Segformer-B5')
            from .encoders.dual_segformer import mit_b5 as backbone
            self.backbone = backbone(norm_fuse=norm_layer)
        elif cfg.backbone == 'mit_b4':
            logger.info('Using backbone: Segformer-B4')
            from .encoders.dual_segformer import mit_b4 as backbone
            self.backbone = backbone(norm_fuse=norm_layer)
        elif cfg.backbone == 'mit_b2':
            logger.info('Using backbone: Segformer-B2')
            from .encoders.dual_segformer import mit_b2 as backbone
            self.backbone = backbone(norm_fuse=norm_layer)
        elif cfg.backbone == 'mit_b1':
            logger.info('Using backbone: Segformer-B1')
            from .encoders.dual_segformer import mit_b0 as backbone
            self.backbone = backbone(norm_fuse=norm_layer)
        elif cfg.backbone == 'mit_b0':
            logger.info('Using backbone: Segformer-B0')
            self.channels = [32, 64, 160, 256]
            from .encoders.dual_segformer import mit_b0 as backbone
            self.backbone = backbone(norm_fuse=norm_layer)
        else:
            logger.info('Using backbone: Segformer-B2')
            from .encoders.dual_segformer import mit_b2 as backbone
            self.backbone = backbone(norm_fuse=norm_layer)
        
        # ��ʼ��Decoder������ԭ���߼���
        if cfg.decoder == 'MLPDecoder':
            from .decoders.MLPDecoder import DecoderHead
            self.decode_head = DecoderHead(in_channels=self.channels, num_classes=cfg.num_classes, 
                                         norm_layer=norm_layer, embed_dim=cfg.decoder_embed_dim)
        elif cfg.decoder == 'UPernet':
            logger.info('Using Upernet Decoder')
            from .decoders.UPernet import UPerHead
            self.decode_head = UPerHead(in_channels=self.channels ,num_classes=cfg.num_classes, norm_layer=norm_layer, channels=512)
            from .decoders.fcnhead import FCNHead
            self.aux_index = 2
            self.aux_rate = 0.4
            self.aux_head = FCNHead(self.channels[2], cfg.num_classes, norm_layer=norm_layer)
        
        elif cfg.decoder == 'deeplabv3+':
            logger.info('Using Decoder: DeepLabV3+')
            from .decoders.deeplabv3plus import DeepLabV3Plus as Head
            self.decode_head = Head(in_channels=self.channels, num_classes=cfg.num_classes, norm_layer=norm_layer)
            from .decoders.fcnhead import FCNHead
            self.aux_index = 2
            self.aux_rate = 0.4
            self.aux_head = FCNHead(self.channels[2], cfg.num_classes, norm_layer=norm_layer)

        else:
            logger.info('No decoder(FCN-32s)')
            from .decoders.fcnhead import FCNHead
            self.decode_head = FCNHead(in_channels=self.channels[-1], kernel_size=3, num_classes=cfg.num_classes, norm_layer=norm_layer)
        
        # ����ͷ��ʼ��
        self.aux_head = None
        if cfg.decoder in ['UPernet', 'deeplabv3+']:
            from .decoders.fcnhead import FCNHead
            self.aux_index = 2
            self.aux_rate = 0.4
            self.aux_head = FCNHead(self.channels[2], cfg.num_classes, norm_layer=norm_layer)

        self.criterion = criterion
        if self.criterion:
            self.init_weights(cfg, pretrained=cfg.pretrained_model)

    def init_weights(self, cfg, pretrained=None):
        # ��ʼ���߼����ֲ���...
        if pretrained:
            self.backbone.init_weights(pretrained=pretrained)
        init_weight(self.decode_head, nn.init.kaiming_normal_,
                   self.norm_layer, cfg.bn_eps, cfg.bn_momentum,
                   mode='fan_in', nonlinearity='relu')
        if self.aux_head:
            init_weight(self.aux_head, nn.init.kaiming_normal_,
                       self.norm_layer, cfg.bn_eps, cfg.bn_momentum,
                       mode='fan_in', nonlinearity='relu')

    def encode_decode(self, rgb, modal_x):
        """�޸ĺ��ǰ�򴫲�����"""
        # ����ԭʼ�ߴ������ϲ���
        orisize = rgb.shape
        
        # ʹ��MFM�ں�RGB�Ͷ��������
        fused_features = self.mfm(rgb, modal_x)  # [B, C, H, W]
        
        # ��������ߴ����
        if fused_features.shape[-2:] != rgb.shape[-2:]:
            fused_features = F.interpolate(fused_features, size=rgb.shape[-2:], mode='bilinear')
        
        # ���ں���������backbone�ĵڶ�����֧
        x = self.backbone(rgb, fused_features)  # ����backbone������������
        
        # ������̱��ֲ���
        out = self.decode_head.forward(x)
        out = F.interpolate(out, size=orisize[2:], mode='bilinear', align_corners=False)
        
        # ����ͷ����
        if self.aux_head:
            aux_fm = self.aux_head(x[self.aux_index])
            aux_fm = F.interpolate(aux_fm, size=orisize[2:], mode='bilinear', align_corners=False)
            return out, aux_fm
        return out

    def forward(self, rgb, modal_x, label=None):
        # ǰ�򴫲��߼����ֲ���
        if self.aux_head:
            out, aux_fm = self.encode_decode(rgb, modal_x)
        else:
            out = self.encode_decode(rgb, modal_x)
        
        if label is not None:
            loss = self.criterion(out, label.long())
            if self.aux_head:
                loss += self.aux_rate * self.criterion(aux_fm, label.long())
            return loss
        return out


# ʵ��MFM������ģ��
class MultiBandFusionModule(nn.Module):
    def __init__(self, rgb_channels=3, ms_channels=25, out_channels=3):
        super().__init__()
        # �ռ�У���׶�
        self.spatial_conv = nn.Sequential(
            nn.Conv2d(rgb_channels + ms_channels, 32, 3, padding=1),
            nn.ReLU(),
            nn.Conv2d(32, 32, 3, padding=1),
            nn.ReLU(),
            nn.MaxPool2d(2),
            nn.Conv2d(32, 64, 3, padding=1),
            nn.ReLU(),
            nn.Conv2d(64, 64, 3, padding=1),
            nn.ReLU(),
            nn.Upsample(scale_factor=2, mode='bilinear', align_corners=True),
            nn.Conv2d(64, 32, 3, padding=1)
        )
        
        # ͨ���ںϽ׶�
        self.channel_fusion = nn.Sequential(
            DenseFusionBlock(32 + rgb_channels + ms_channels, 24),
            DenseFusionBlock(24, 12),
            DenseFusionBlock(12, 6),
            DenseFusionBlock(6, out_channels)
        )

    def forward(self, rgb, ms):
        # �ռ����
        if ms.shape[-2:] != rgb.shape[-2:]:
            ms = F.interpolate(ms, size=rgb.shape[-2:], mode='bilinear')
        
        # ����ƴ��
        x = torch.cat([rgb, ms], dim=1)
        
        # �ռ�У��
        spatial_feat = self.spatial_conv(x)
        
        # ͨ���ں�
        fused = torch.cat([spatial_feat, x], dim=1)
        return self.channel_fusion(fused)

class DenseFusionBlock(nn.Module):
    def __init__(self, in_channels, out_channels):
        super().__init__()
        self.conv = nn.Sequential(
            nn.Conv2d(in_channels, out_channels, 1),
            nn.BatchNorm2d(out_channels),
            nn.ReLU()
        )
        self.se = SqueezeExcitation(out_channels)
        self.transition = nn.Conv2d(in_channels + out_channels, out_channels, 1)

    def forward(self, x):
        identity = x
        x = self.conv(x)
        x = self.se(x)
        x = torch.cat([identity, x], dim=1)
        return self.transition(x)

class SqueezeExcitation(nn.Module):
    def __init__(self, channel, reduction=4):
        super().__init__()
        self.avgpool = nn.AdaptiveAvgPool2d(1)
        self.fc = nn.Sequential(
            nn.Linear(channel, channel // reduction),
            nn.ReLU(),
            nn.Linear(channel // reduction, channel),
            nn.Sigmoid()
        )

    def forward(self, x):
        b, c, _, _ = x.size()
        y = self.avgpool(x).view(b, c)
        y = self.fc(y).view(b, c, 1, 1)
        return x * y.expand_as(x)