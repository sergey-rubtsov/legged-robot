import argparse
import os
import torch


from exp.exp_informer import Exp_Informer

Exp = Exp_Informer

args = argparse.Namespace(model='informer',
                          data='ETTh1',
                          root_path='./data/ETT/',
                          data_path='ETTh1.csv',
                          features='M',
                          target='OT',
                          freq='h',
                          checkpoints='./checkpoints/',
                          seq_len=96,
                          label_len=48,
                          pred_len=24,
                          enc_in=7,
                          dec_in=7,
                          c_out=7,
                          d_model=512,
                          n_heads=8,
                          e_layers=2,
                          d_layers=1,
                          s_layers=[3, 2, 1],
                          d_ff=2048,
                          factor=5,
                          padding=0,
                          distil=True,
                          dropout=0.05,
                          attn='prob',
                          embed='timeF',
                          activation='gelu',
                          output_attention=False,
                          do_predict=False,
                          mix=True,
                          cols=None,
                          num_workers=0,
                          itr=1,
                          train_epochs=6,
                          batch_size=32,
                          patience=3,
                          learning_rate=0.0001,
                          des='test',
                          loss='mse',
                          lradj='type1',
                          use_amp=False,
                          inverse=False,
                          use_gpu=False,
                          gpu=0,
                          use_multi_gpu=False,
                          devices='0,1,2,3',
                          detail_freq='h')

print('Args in experiment:')
print(args)

setting = '{}_{}_ft{}_sl{}_ll{}_pl{}_dm{}_nh{}_el{}_dl{}_df{}_at{}_fc{}_eb{}_dt{}_mx{}_{}_{}'.format(args.model,
                                                                                                     args.data,
                                                                                                     args.features,
                                                                                                     args.seq_len,
                                                                                                     args.label_len,
                                                                                                     args.pred_len,
                                                                                                     args.d_model,
                                                                                                     args.n_heads,
                                                                                                     args.e_layers,
                                                                                                     args.d_layers,
                                                                                                     args.d_ff,
                                                                                                     args.attn,
                                                                                                     args.factor,
                                                                                                     args.embed,
                                                                                                     args.distil,
                                                                                                     args.mix,
                                                                                                     args.des,
                                                                                                     args.itr)

exp = Exp(args) # set experiments
print('>>>>>>>start training : {}>>>>>>>>>>>>>>>>>>>>>>>>>>'.format(setting))
exp.train(setting)
# print('>>>>>>>testing : {}<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<'.format(setting))
# exp.test(setting)
print('>>>>>>>predicting : {}<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<'.format(setting))
exp.predict(setting, True)
torch.cuda.empty_cache()
