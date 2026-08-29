//! Visibility graph leftover. Upstream `AP_OAVisGraph`.
//!
//! Dijkstra (`AP_OADijkstra`) stores fence-to-fence, source-to-nodes, and
//! destination-to-nodes edges here. ADR-0004 forbids the expanding-array
//! allocator; this leftover uses a fixed table.

/// Max edges in one leftover graph. Upstream grows by 20 via `AP_ExpandingArray`.
pub const VISGRAPH_ITEMS_MAX: usize = 280;

/// Item flavour. Upstream `AP_OAVisGraph::OAType`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OaType {
    /// Current / start. Upstream `OATYPE_SOURCE`.
    Source = 0,
    /// Mission destination. Upstream `OATYPE_DESTINATION`.
    Destination = 1,
    /// Fence-margin vertex. Upstream `OATYPE_INTERMEDIATE_POINT`.
    IntermediatePoint = 2,
}

/// Unique node id. Upstream `AP_OAVisGraph::OAItemID`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct OaItemId {
    /// Flavour.
    pub id_type: OaType,
    /// Index within that flavour. Upstream `oaid_num`.
    pub id_num: u8,
}

impl OaItemId {
    /// Source node (always id 0).
    #[must_use]
    pub const fn source() -> Self {
        Self {
            id_type: OaType::Source,
            id_num: 0,
        }
    }

    /// Destination node (always id 0).
    #[must_use]
    pub const fn destination() -> Self {
        Self {
            id_type: OaType::Destination,
            id_num: 0,
        }
    }

    /// Fence-margin vertex `id_num`.
    #[must_use]
    pub const fn intermediate(id_num: u8) -> Self {
        Self {
            id_type: OaType::IntermediatePoint,
            id_num,
        }
    }
}

/// One visibility edge. Upstream `AP_OAVisGraph::VisGraphItem`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VisGraphItem {
    /// First endpoint.
    pub id1: OaItemId,
    /// Second endpoint.
    pub id2: OaItemId,
    /// Distance between the endpoints, centimetres.
    pub distance_cm: f32,
}

impl Default for VisGraphItem {
    fn default() -> Self {
        Self {
            id1: OaItemId::source(),
            id2: OaItemId::source(),
            distance_cm: 0.0,
        }
    }
}

/// Visibility graph leftover. Upstream `AP_OAVisGraph`.
#[derive(Debug, Clone)]
pub struct VisGraph {
    items: [VisGraphItem; VISGRAPH_ITEMS_MAX],
    num_items: u16,
}

impl Default for VisGraph {
    fn default() -> Self {
        Self::new()
    }
}

impl VisGraph {
    /// Empty graph.
    #[must_use]
    pub fn new() -> Self {
        Self {
            items: [VisGraphItem::default(); VISGRAPH_ITEMS_MAX],
            num_items: 0,
        }
    }

    /// Leftover of `AP_OAVisGraph::clear`.
    pub fn clear(&mut self) {
        self.num_items = 0;
    }

    /// Populated edge count. Upstream `num_items`.
    #[must_use]
    pub fn num_items(&self) -> u16 {
        self.num_items
    }

    /// Edge `i`, or `None` when out of range.
    #[must_use]
    pub fn item(&self, i: u16) -> Option<VisGraphItem> {
        if usize::from(i) >= usize::from(self.num_items) {
            return None;
        }
        self.items.get(usize::from(i)).copied()
    }

    /// Leftover of `AP_OAVisGraph::add_item`.
    ///
    /// Returns `false` when the table is full (upstream expanding-array OOM).
    pub fn add_item(&mut self, id1: OaItemId, id2: OaItemId, distance_cm: f32) -> bool {
        if self.num_items == u16::MAX {
            return false;
        }
        let idx = usize::from(self.num_items);
        let Some(slot) = self.items.get_mut(idx) else {
            return false;
        };
        *slot = VisGraphItem {
            id1,
            id2,
            distance_cm,
        };
        self.num_items = self.num_items.saturating_add(1);
        true
    }
}
