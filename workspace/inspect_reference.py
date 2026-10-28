#!/usr/bin/env python3
"""Inspect published ACT model config and training dataset metadata without training.

Downloads JSON metadata only; it does not download entire datasets or weights.
"""
import argparse
import json

from huggingface_hub import hf_hub_download


def fetch(repo, file, kind):
    try:
        path = hf_hub_download(repo_id=repo, filename=file, repo_type=kind)
    except Exception as exc:
        print(f'UNAVAILABLE {repo}/{file}: {type(exc).__name__}: {exc}')
        return None
    with open(path, encoding='utf-8') as f:
        return json.load(f)


def show_feature(name, info):
    if isinstance(info, dict):
        print(f'  {name}: shape={info.get("shape")} dtype={info.get("dtype")} type={info.get("type")}')
    else:
        print(f'  {name}: {info}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', default='shraavb/act_pick_place_cube')
    parser.add_argument('--dataset', default='shraavb/pick_place_cube')
    args = parser.parse_args()
    config = fetch(args.model, 'config.json', 'model')
    if config:
        print('\nMODEL:', args.model)
        print('type:', config.get('type'))
        print('input_features:')
        for k,v in config.get('input_features', {}).items(): show_feature(k,v)
        print('output_features:')
        for k,v in config.get('output_features', {}).items(): show_feature(k,v)
        print('chunk_size:',config.get('chunk_size'))
    info = fetch(args.dataset, 'meta/info.json', 'dataset')
    if info:
        print('\nDATASET:', args.dataset)
        print('fps:', info.get('fps'))
        for k,v in info.get('features', {}).items(): show_feature(k,v)
    stats = fetch(args.dataset, 'meta/stats.json', 'dataset')
    if stats:
        print('\nDATASET STATS (NOT PHYSICAL JOINT LIMITS)')
        for name in ('observation.state', 'action'):
            if name in stats:
                print(name)
                for field in ('min', 'max', 'mean', 'std'):
                    if field in stats[name]: print(' ',field,':',stats[name][field])
    print('\nNOTE: dataset statistics are not sufficient to infer joint zeros/directions or camera extrinsics.')


if __name__ == '__main__': main()
