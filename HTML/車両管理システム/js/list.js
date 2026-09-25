import { db } from "./firebase.js";
import { logout, requireAuth } from "./auth.js";
import { collection, onSnapshot } from "https://www.gstatic.com/firebasejs/12.19.0/firebase-firestore.js";

let allBikes = [];
let currentFilter = 'all';

// 未ログインならログインページへ
requireAuth(() => { window.location.href = 'index.html'; });

// Firestoreからリアルタイム取得
onSnapshot(collection(db, 'CycleInfo'), (snapshot) => {
  allBikes = snapshot.docs.map(doc => ({ id: doc.id, ...doc.data() }));
  renderTable(getFilteredBikes());
  updateSummary();
});

// タイムスタンプを文字列に変換
function formatTimestamp(ts) {
  if (!ts) return '-';
  if (ts.toDate) {
    const d = ts.toDate();
    return `${d.getFullYear()}-${String(d.getMonth()+1).padStart(2,'0')}-${String(d.getDate()).padStart(2,'0')} ${String(d.getHours()).padStart(2,'0')}:${String(d.getMinutes()).padStart(2,'0')}`;
  }
  return String(ts);
}

// サマリー更新
function updateSummary() {
  const total = allBikes.length;
  const maintenance = allBikes.filter(b => b.status === 'maintenance_required').length;
  document.getElementById('total-count').textContent = total;
  document.getElementById('maintenance-count').textContent = maintenance;
  document.getElementById('normal-count').textContent = total - maintenance;
}

// フィルター
function getFilteredBikes() {
  if (currentFilter === 'all') return allBikes;
  if (currentFilter === 'maintenance') return allBikes.filter(b => b.status === 'maintenance_required');
  return allBikes.filter(b => b.status === 'normal');
}

window.filterBikes = function(filter, btn) {
  currentFilter = filter;
  document.querySelectorAll('.filter-btn').forEach(b => b.classList.remove('active'));
  btn.classList.add('active');
  renderTable(getFilteredBikes());
};

// テーブル描画
function renderTable(data) {
  const tbody = document.getElementById('bike-table');
  tbody.innerHTML = '';

  if (data.length === 0) {
    tbody.innerHTML = '<tr><td colspan="4" class="loading">該当する車両がありません</td></tr>';
    return;
  }

  data.forEach(bike => {
    const isMaintenance = bike.status === 'maintenance_required';
    const tr = document.createElement('tr');
    tr.innerHTML = `
      <td>${bike.id}</td>
      <td><span class="badge ${isMaintenance ? 'maintenance' : 'normal'}">${isMaintenance ? '要メンテナンス' : '正常'}</span></td>
      <td>${formatTimestamp(bike.last_synced)}</td>
      <td>${formatTimestamp(bike.last_maintenance)}</td>
    `;
    tr.onclick = () => {
      // 詳細ページへ車両IDを渡す
      sessionStorage.setItem('selectedBikeId', bike.id);
      window.location.href = 'detail.html';
    };
    tbody.appendChild(tr);
  });
}

// ログアウト
window.handleLogout = async function() {
  await logout();
  window.location.href = 'index.html';
};
