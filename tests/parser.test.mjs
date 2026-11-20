// URDF parser edge cases (urdfdom-compatible behaviour).
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { DOMParser } from '@xmldom/xmldom';
import { parseURDF, controllableJoints } from '../js/urdf/parser.js';
import { KinematicChain } from '../js/kinematics/chain.js';
import { defaultTipLink } from '../js/urdf/robot.js';
import { parseRef } from '../js/urdf/assets.js';

const parse = (xml) => parseURDF(xml, { DOMParser });
const link = (n, extra = '') => `<link name="${n}">${extra}</link>`;
const joint = (n, type, p, c, extra = '') =>
  `<joint name="${n}" type="${type}"><parent link="${p}"/><child link="${c}"/><axis xyz="0 0 1"/>${extra}</joint>`;
const lim = '<limit lower="-1" upper="1" effort="1" velocity="1"/>';

test('flattened URDF with xmlns:xacro and a commented-out xacro tag is accepted', () => {
  const m = parse(`<robot name="r" xmlns:xacro="http://ros.org/wiki/xacro">
    <!-- <xacro:include filename="$(find gripper)/g.xacro"/> ${'${height}'} -->
    ${link('a')}${link('b')}${joint('j', 'revolute', 'a', 'b', lim)}</robot>`);
  assert.equal(controllableJoints(m).length, 1);
  assert.ok(!m.warnings.some((w) => /xacro/.test(w)));
});

test('real xacro is still rejected with a helpful message', () => {
  assert.throws(() => parse(`<robot name="r" xmlns:xacro="http://ros.org/wiki/xacro"><xacro:include filename="x.xacro"/></robot>`), /xacro/);
});

test('leading whitespace before <?xml is tolerated', () => {
  const m = parse(`\n  <?xml version="1.0"?>\n<robot name="r">${link('a')}</robot>`);
  assert.equal(m.root, 'a');
});

test('undeclared namespace prefixes in gazebo blocks are tolerated', () => {
  const m = parse(`<robot name="r">${link('a')}<gazebo><sensor:camera name="c"/></gazebo></robot>`);
  assert.equal(m.root, 'a');
});

test('named inline material is reusable by name; an existing global wins over inline values', () => {
  const vis = (mat) => `<visual><geometry><box size="1 1 1"/></geometry>${mat}</visual>`;
  const m = parse(`<robot name="r">
    <material name="red"><color rgba="1 0 0 1"/></material>
    ${link('a', vis('<material name="blue"><color rgba="0 0 1 1"/></material>'))}
    ${link('b', vis('<material name="blue"/>'))}
    ${link('c', vis('<material name="red"><color rgba="0 1 0 1"/></material>'))}
    ${joint('j1', 'fixed', 'a', 'b')}${joint('j2', 'fixed', 'a', 'c')}</robot>`);
  assert.deepEqual(m.links.get('b').visuals[0].material.color, [0, 0, 1, 1]);
  assert.deepEqual(m.links.get('c').visuals[0].material.color, [1, 0, 0, 1]);
});

test('mimic resolution does not depend on joint order', () => {
  const body = (order) => {
    const jA = joint('A', 'revolute', 'l0', 'l1', `${lim}<mimic joint="B" multiplier="2"/>`);
    const jB = joint('B', 'revolute', 'l1', 'l2', `${lim}<mimic joint="C"/>`); // C does not exist
    return `<robot name="r">${link('l0')}${link('l1')}${link('l2')}${order === 'AB' ? jA + jB : jB + jA}</robot>`;
  };
  for (const order of ['AB', 'BA']) {
    const m = parse(body(order));
    assert.equal(m.joints.get('A').mimicResolved?.master, 'B', `order ${order}`);
    assert.equal(m.joints.get('B').mimicResolved, undefined);
  }
});

test('default end-effector skips gripper joints and prefers named tool frames', () => {
  const m = parse(`<robot name="r">${link('base')}${link('l1')}${link('tool0')}${link('finger')}
    ${joint('j1', 'revolute', 'base', 'l1', lim)}${joint('t', 'fixed', 'l1', 'tool0')}
    ${joint('slide', 'prismatic', 'l1', 'finger', lim)}</robot>`);
  assert.equal(defaultTipLink(m), 'tool0');
});

test('periodic flag is false for a master driving a prismatic mimic follower', () => {
  const m = parse(`<robot name="r">${link('a')}${link('b')}${link('c')}
    ${joint('turn', 'continuous', 'a', 'b')}${joint('rack', 'prismatic', 'b', 'c', `${lim}<mimic joint="turn" multiplier="0.05"/>`)}</robot>`);
  const chain = new KinematicChain(m, 'a', 'c');
  assert.deepEqual(chain.periodic, [false]);
});

test('mesh references: backslashes, absolute and Windows paths', () => {
  assert.deepEqual(parseRef('package://pkg\\meshes\\a.stl'), { scheme: 'package', pkg: 'pkg', path: 'meshes/a.stl' });
  assert.equal(parseRef('/home/u/ws/src/pkg/meshes/a.stl').scheme, 'absolute');
  assert.equal(parseRef('C:\\ws\\pkg\\meshes\\a.stl').scheme, 'file');
  assert.equal(parseRef('file:///home/u/100%/pkg/a.stl').path, 'home/u/100%/pkg/a.stl');
});
