const { test } = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const source = fs.readFileSync(require('node:path').join(__dirname, '../app.js'), 'utf8');
const refresh = source.slice(source.indexOf('async function refreshPipFrame('), source.indexOf('\nfunction actionButton('));

function fixture() {
  const elements = {};
  const calls = [];
  const context = {
    state: { scene: 'office', generation: 1, frame_id: 2, head_frame_available: true, depth_frame_available: true },
    pipFrames: { head: { id: -1, url: '' }, depth: { id: -1, url: '' } },
    $: key => elements[key] ??= { open: true },
    AbortSignal,
    URL: { createObjectURL: () => 'blob:frame', revokeObjectURL: () => {} },
    fetch: async url => {
      calls.push(url);
      return { ok: true, blob: async () => ({}), headers: new Map([['X-Scene-Generation', '1'], ['X-Frame-Id', '2']]) };
    },
  };
  vm.createContext(context);
  vm.runInContext(refresh, context);
  return { context, calls };
}

test('each collapsed pip stops only its own downloads', async () => {
  for (const closed of ['head', 'depth']) {
    const { context, calls } = fixture();
    context.$(`#${closed}-pip`).open = false;
    await Promise.all(['head', 'depth'].map(kind => context.refreshPipFrame(kind)));
    assert.equal(calls.length, 1);
    assert.ok(!calls[0].includes(closed));
  }
});

test('old scene images are discarded; successful frames are not downloaded twice', async () => {
  const { context, calls } = fixture();
  context.state.generation = 3;
  await context.refreshPipFrame('depth');
  assert.equal(context.pipFrames.depth.id, -1);
  context.state.generation = 1;
  await context.refreshPipFrame('depth');
  await context.refreshPipFrame('depth');
  assert.equal(calls.length, 2);
  assert.equal(context.pipFrames.depth.id, 2);
});
