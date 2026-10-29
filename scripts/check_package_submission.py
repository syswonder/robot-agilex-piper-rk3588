#!/usr/bin/env python3
"""Apply the upstream catalog's strict new-entry validation to local source.

Only remote filesystem reads are adapted. Metadata, contracts, IDL, transport,
artwork, deployment resolution and new-entry strictness use upstream functions.
This does not assert that a remote default branch has been published.
"""
import argparse
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import yaml
from catalog_common import ROOT, source_files

PIN = '65cf91c32acb52e0ec72ff7ef4a003c02c78f00f'

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--catalog-source', type=Path, required=True)
    parser.add_argument('--root', type=Path, default=ROOT)
    parser.add_argument('--robonix-source', type=Path, default=ROOT/'vendor/robonix')
    parser.add_argument('--repo', default='https://github.com/syswonder/robot-agilex-piper-rk3588')
    parser.add_argument('--output', type=Path, default=ROOT/'dist/catalog')
    args = parser.parse_args()
    root = args.root.resolve()
    if sys.version_info < (3, 11):
        import tomli
        sys.modules['tomllib'] = tomli
    revision = subprocess.check_output(['git','-C',str(args.catalog_source),'rev-parse','HEAD'], text=True).strip()
    if revision != PIN:
        raise SystemExit(f'Catalog checker revision is {revision}; expected {PIN}')
    spec = importlib.util.spec_from_file_location('upstream_catalog', args.catalog_source/'scripts/build_catalog.py')
    official = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(official)
    official.parse_repo(args.repo)
    files = source_files(root)
    tree = {}
    for rel in files:
        tree[rel] = 'blob'
        for parent in Path(rel).parents:
            if str(parent) != '.':
                tree[parent.as_posix()] = 'tree'
    def read(rel):
        if rel not in tree or tree[rel] != 'blob':
            raise FileNotFoundError(f'File is absent from publication inventory: {rel}')
        return (root/rel).read_text()
    def manifest(repo, rel, path=''):
        prefix = f'{path}/' if path else ''
        return 'main', yaml.safe_load(read(prefix+rel)), read(prefix+'README.md')
    def artwork(repo, branch, path=''):
        prefix = f'{path}/' if path else ''
        paths = [p[len(prefix):] for p in files if p.startswith(prefix+'assets/')]
        result = official.check_artwork(paths, lambda p: (root/prefix/p).read_bytes())
        url = lambda p: f'https://raw.githubusercontent.com/{official.parse_repo(repo)[0]}/{official.parse_repo(repo)[1]}/{branch}/{prefix}{p}'
        chosen = result['promo'].get('', [])
        return {'promo': [url(p) for p in chosen], 'icon': '',
                'problems': result['problems'],
                'data': {url(p): data for p, data in result['data'].items()}}
    official.load_remote_manifest = manifest
    official.load_remote_repository_tree = lambda repo, branch: tree
    official.load_remote_default_branch = lambda repo: 'main'
    official.load_remote_images = artwork
    official.raw_text = lambda repo, branch, path: read(path)
    # Runtime contracts come from the pinned Robonix source used by this robot.
    caps_root = args.robonix_source/'capabilities'
    if not caps_root.is_dir():
        raise SystemExit('Missing Robonix contracts; fetch vendor/robonix as documented in vendor/PINS.md')
    contracts = [official.contract_of(p.read_text()) for p in caps_root.rglob('*.toml')
                 if 'lib' not in p.relative_to(caps_root).parts]
    ids = {c['id'] for c in contracts if c}
    lib = {p.relative_to(caps_root/'lib').as_posix() for p in (caps_root/'lib').rglob('*') if p.is_file()}
    official._robonix_contracts = (ids, lib, ())
    paths = json.loads(read('catalog/package-paths.json'))
    packages = []
    for path in paths:
        meta = yaml.safe_load(read(path+'/package_manifest.yaml'))['package']
        if not any(meta['name'].startswith(f'robonix.{kind}.') for kind in official.KINDS):
            raise SystemExit(f'Package name must use a Robonix kind prefix: {meta["name"]}')
        packages.append({'name':meta['name'], 'repo':args.repo, 'path':path})
    robot = yaml.safe_load(read('robonix_manifest.yaml'))['catalog']['name']
    fragment = {'packages': packages, 'robots':[{'name':robot, 'repo':args.repo}]}
    entries = official.parse_catalog(fragment)
    names = [e['name'] for e in entries]
    if len(set(names)) != len(names):
        raise SystemExit('Duplicate catalog names')
    baseline = yaml.safe_load((args.catalog_source/'catalog.yaml').read_text())
    existing = {e['name'] for key in ('packages','robots') for e in baseline[key]}
    if existing.intersection(names):
        raise SystemExit(f'Names already registered: {sorted(existing.intersection(names))}')
    records = [official.collect_entry(e, set()) for e in entries]
    official.annotate_deploy_dependencies(records)
    official.validate_deploy_dependency_paths(records)
    problems = [p for r in records for p in r.get('catalog_warnings', [])+r.get('deployment_warnings', [])+r.get('_artwork_problems', [])]
    if problems:
        raise SystemExit(json.dumps(problems, indent=2))
    # Confirm every relative dependency has a checked bundled package manifest.
    by_path = {r.get('path'):r['name'] for r in records if r['catalog_type']=='package'}
    for r in records:
        for dep in r['deploy_dependencies']:
            if dep['path'] not in by_path:
                raise SystemExit(f'Deployment dependency not checked: {dep["path"]}')
    # Whole-robot submissions may bundle dependencies without separately listing
    # them in the catalog. Validate this exact submission mode as well.
    robot_records = [r for r in records if r['catalog_type'] == 'robot']
    official.annotate_deploy_dependencies(robot_records)
    official.validate_deploy_dependency_paths(robot_records)
    robot_problems = [p for r in robot_records for p in r.get('deployment_warnings', [])]
    if robot_problems:
        raise SystemExit(json.dumps(robot_problems, indent=2))
    submission = {'robots': fragment['robots']}
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output/'catalog.additions.yaml').write_text(yaml.safe_dump(submission,sort_keys=False))
    report = {'status':'passed', 'scope':'local source with official strict new-entry rules',
              'catalog_checker_commit':revision, 'remote_verified':False,
              'repository_url_confirmed':'/OWNER/' not in args.repo,
              'packages_checked':len(packages), 'robots':1, 'submitted_catalog_entries':1, 'deployment_dependencies':sum(len(r['deploy_dependencies']) for r in records),
              'files':len(files), 'warnings':problems,
              'contracts_source':str(args.robonix_source),
              'entries':[{k:v for k,v in r.items() if not k.startswith('_')} for r in records]}
    (args.output/'validation.json').write_text(json.dumps(report, indent=2, ensure_ascii=False)+'\n')
    print(f'PASS: 1 whole-robot submission; {len(packages)} bundled packages checked; all dependency paths, metadata, contracts, IDL, transports and artwork pass.')
    print(f'Catalog additions and report: {args.output}')
    if '/OWNER/' in args.repo:
        print('Remote URL pending: replace OWNER by rerunning with --repo https://github.com/<owner>/<repo>.')

if __name__ == '__main__':
    main()
