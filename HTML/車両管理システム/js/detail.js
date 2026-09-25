import { db } from "./firebase.js";
import { logout, requireAuth } from "./auth.js";
import { doc, onSnapshot, updateDoc, Timestamp } from "https://www.gstatic.com/firebasejs/12.19.0/firebase-firestore.js";

// 未ログインならログインページへ
requireAuth(() => { window.location.href = 'index.html'; });

// sessionStorageから車両IDを取得
const bikeId = sessionStorage.getItem('selectedBikeId');
if (!bikeId) window.location.href = 'list.html';

// タイムスタンプを文字列に変換
function formatTimestamp(ts) {
  if (!ts) return '-';
  if (ts.toDate) {
    const d = ts.toDate();
    return `${d.getFullYear()}-${String(d.getMonth()+1).padStart(2,'0')}-${String(d.getDate()).padStart(2,'0')} ${String(d.getHours()).padStart(2,'0')}:${String(d.getMinutes()).padStart(2,'0')}`;
  }
  return String(ts);
}

// Firestoreからリアルタイム取得
onSnapshot(doc(db, 'CycleInfo', bikeId), (snapshot) => {
  if (!snapshot.exists()) {
    alert('車両データが見つかりません');
    window.location.href = 'list.html';
    return;
  }
  const bike = { id: snapshot.id, ...snapshot.data() };
  renderDetail(bike);
});

// 詳細を描画
function renderDetail(bike) {
  const isMaintenance = bike.status === 'maintenance_required';

  document.getElementById('d-id').textContent = bike.id;
  document.getElementById('d-status').innerHTML = `<span class="badge ${isMaintenance ? 'maintenance' : 'normal'}">${isMaintenance ? '要メンテナンス' : '正常'}</span>`;
  document.getElementById('d-vibration').textContent = bike.vibration_level || '-';
  document.getElementById('d-report').textContent = bike.user_report ? 'あり' : 'なし';
  document.getElementById('d-synced').textContent = formatTimestamp(bike.last_synced);
  document.getElementById('d-maintenance').textContent = formatTimestamp(bike.last_maintenance);

  document.getElementById('restore-area').style.display = isMaintenance ? 'flex' : 'none';
  document.getElementById('restored-msg').style.display = 'none';

  const btn = document.getElementById('restore-btn');
  if (btn) {
    btn.disabled = false;
    btn.textContent = '点検完了・復旧する';
  }
}

// 復旧処理
window.restore = async function() {
  const btn = document.getElementById('restore-btn');
  btn.disabled = true;
  btn.textContent = '更新中...';

  try {
    await updateDoc(doc(db, 'CycleInfo', bikeId), {
      status: 'normal',
      vibration_level: 'none',
      user_report: false,
      last_maintenance: Timestamp.now()
    });
    document.getElementById('restore-area').style.display = 'none';
    document.getElementById('restored-msg').style.display = 'block';
  } catch (e) {
    alert('復旧に失敗しました。もう一度お試しください。');
    btn.disabled = false;
    btn.textContent = '点検完了・復旧する';
  }
};

// ログアウト
window.handleLogout = async function() {
  await logout();
  window.location.href = 'index.html';
};
