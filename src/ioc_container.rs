use once_cell::sync::OnceCell;
use std::{
    any::{Any, TypeId},
    collections::HashMap,
    sync::{Arc, Mutex},
};
use thiserror::Error;

#[derive(Error, Debug)]
pub enum IocContainerError {
    #[error("service not available in container")]
    ServiceNotAvailable,
    #[error("failed to downcast service")]
    FailedToDowncast,
}

type Handle = Arc<dyn Any + Send + Sync>;

#[derive(Debug, Clone, Default)]
pub struct IocContainer {
    map: Arc<Mutex<HashMap<TypeId, Handle>>>,
}

impl IocContainer {
    pub fn register<T: Any + Send + Sync>(&self, object: T) {
        let type_id = object.type_id();
        self.map.lock().unwrap().insert(type_id, Arc::new(object));
    }

    pub fn get<T: Any + Send + Sync>(&self) -> Option<Arc<T>> {
        let type_id = TypeId::of::<T>();
        if let Some(object) = self.map.lock().unwrap().get(&type_id) {
            let handle = object.clone();
            handle.downcast::<T>().ok()
        } else {
            None
        }
    }

    pub fn service<T: Any + Send + Sync>(&self) -> Result<Arc<T>, IocContainerError> {
        let type_id = TypeId::of::<T>();
        if let Some(object) = self.map.lock().unwrap().get(&type_id) {
            let handle = object.clone();
            handle
                .downcast::<T>()
                .map_err(|_| IocContainerError::FailedToDowncast)
        } else {
            Err(IocContainerError::ServiceNotAvailable)
        }
    }

    pub fn global_instance() -> &'static IocContainer {
        static INSTANCE: OnceCell<IocContainer> = OnceCell::new();
        INSTANCE.get_or_init(IocContainer::default)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct A;

    #[test]
    fn simple_ioc() {
        let container = IocContainer::default();
        container.register(A);
        let a = container.get::<A>();
        assert!(a.is_some());
    }

    #[test]
    fn fail_on_unregistered_type() {
        let container = IocContainer::default();
        let not_a = container.get::<A>();
        assert!(not_a.is_none())
    }
}
